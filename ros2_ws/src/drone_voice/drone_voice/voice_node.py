#!/usr/bin/env python3
'''K1 语音交互 ROS2 节点 — VAD→ASR→LLM→TTS 管道, 发布命令到 /drone/command'''
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import sys, os, time, subprocess, threading, re
import numpy as np

os.environ['JACK_NO_START_SERVER'] = '1'

sys.path.insert(0, os.path.expanduser('~/spacemit-demo/examples/NLP'))
sys.path.insert(0, os.path.expanduser('~'))
from llm_server import chat
from spacemit_asr import ASRModel

CMD_SYSTEM_PROMPT = '''你是K1无人机语音助手，用简体中文简短回答，不超过25字。
当用户要求执行飞行动作时，在回复末尾添加命令标签：
  [CMD:ARM] 解锁 / [CMD:TAKEOFF] 起飞 / [CMD:LAND] 降落 / [CMD:RTL] 返航'''


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/drone/command', 10)
        self.text_pub = self.create_publisher(String, '/drone/voice_text', 10)

        # State
        self._running = True
        self._ready = False

        # Init heavy models in background so ROS2 doesn't timeout
        self._init_thread = threading.Thread(target=self._init_pipeline, daemon=True)
        self._init_thread.start()

        # Heartbeat timer until ready
        self._hb_timer = self.create_timer(2.0, self._heartbeat)

    def _heartbeat(self):
        if not self._ready:
            self.get_logger().info('Initializing voice pipeline...')
        else:
            self.destroy_timer(self._hb_timer)

    def _init_pipeline(self):
        '''Load ASR, VAD, configure audio — runs once in background thread.'''
        try:
            self.get_logger().info('Loading ASR model...')
            self.asr = ASRModel()
            self.get_logger().info('ASR model loaded')

            # Audio routing
            subprocess.run(['amixer', '-c', '1', 'cset', 'numid=9', '8'],
                           capture_output=True)
            subprocess.run(['amixer', '-c', '1', 'cset', 'numid=8', '3,3'],
                           capture_output=True)

            r = subprocess.run(['pactl', 'list', 'short', 'sources'],
                               capture_output=True, text=True)
            for line in r.stdout.strip().split('\n'):
                if 'alsa_input.platform-snd-card_1' in line:
                    subprocess.run(['pactl', 'set-default-source', line.split()[0]],
                                   capture_output=True)
                    break

            r2 = subprocess.run(['pactl', 'list', 'short', 'sinks'],
                                capture_output=True, text=True)
            for line in r2.stdout.strip().split('\n'):
                if 'USB_Audio' in line:
                    subprocess.run(['pactl', 'set-default-sink', line.split()[0]],
                                   capture_output=True)
                    break

            # VAD
            from scipy.signal import resample
            import onnxruntime as ort
            from collections import deque
            self.resample = resample
            self.deque = deque

            vad_path = os.path.expanduser('~/.cache/sensevoice/silero_vad.onnx')
            self.vad_sess = ort.InferenceSession(vad_path, providers=['CPUExecutionProvider'])
            self.get_logger().info('VAD model loaded')

            # Find input device — suppress ALSA probe noise
            import pyaudio
            stderr_save = sys.stderr
            sys.stderr = open('/dev/null', 'w')
            try:
                pa = pyaudio.PyAudio()
            finally:
                sys.stderr.close()
                sys.stderr = stderr_save
            self._pa = pa  # keep alive for voice loop
            self.INPUT_INDEX = None
            for i in range(pa.get_device_count()):
                info = pa.get_device_info_by_index(i)
                if info.get('maxInputChannels', 0) > 0 and 'default' in info.get('name', ''):
                    self.INPUT_INDEX = i
                    self.DEV_RATE = int(info.get('defaultSampleRate', 48000))
                    break
            if self.INPUT_INDEX is None:
                for i in range(pa.get_device_count()):
                    info = pa.get_device_info_by_index(i)
                    if info.get('maxInputChannels', 0) > 0 and 'pipewire' in info.get('name', ''):
                        self.INPUT_INDEX = i
                        self.DEV_RATE = int(info.get('defaultSampleRate', 48000))
                        break

            if self.INPUT_INDEX is None:
                self.get_logger().error('No capture device found')
                return

            self.FRAME_SIZE = 1536  # 32ms @ 48kHz
            self.get_logger().info(
                f'Voice pipeline ready (dev={self.INPUT_INDEX}, rate={self.DEV_RATE})')
            self._ready = True

            # Start voice detection loop
            self._voice_loop()

        except Exception as e:
            self.get_logger().error(f'Init failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _voice_loop(self):
        '''Main voice detection loop — runs in background thread.'''
        import pyaudio

        while self._running:
            try:
                stream = self._pa.open(
                    format=pyaudio.paInt16, channels=1, rate=self.DEV_RATE,
                    input=True, frames_per_buffer=self.FRAME_SIZE,
                    input_device_index=self.INPUT_INDEX)

                vad_state = np.zeros((2, 1, 128), dtype=np.float32)
                vad_ctx = np.zeros((1, 64), dtype=np.float32)
                vad_sr = np.array(16000, dtype=np.int64)
                pre_buffer = self.deque(maxlen=20)
                frames = []
                recording = False
                silent_count = 0
                SILENCE_MAX = int(1.2 * self.DEV_RATE / self.FRAME_SIZE)
                t_start = time.time()

                while self._running:
                    frame = stream.read(self.FRAME_SIZE, exception_on_overflow=False)

                    frame_np = np.frombuffer(frame, dtype=np.int16).astype(np.float32)
                    n_16k = int(len(frame_np) * 16000 / self.DEV_RATE)
                    frame_16k = self.resample(frame_np, n_16k).astype(np.float32) / 32768.0

                    x = np.concatenate((vad_ctx, frame_16k[np.newaxis, :]), axis=1)
                    vad_ctx = x[:, -64:]
                    prob, vad_state = self.vad_sess.run(
                        None,
                        {'input': x.astype(np.float32), 'state': vad_state, 'sr': vad_sr})
                    prob = float(prob)

                    pre_buffer.append(frame)

                    if prob > 0.45 and not recording:
                        recording = True
                        silent_count = 0
                        frames = list(pre_buffer)
                        self.get_logger().info('Speech detected')

                    if recording:
                        frames.append(frame)
                        if prob < 0.25:
                            silent_count += 1
                        else:
                            silent_count = 0
                        if silent_count > SILENCE_MAX:
                            break

                    if time.time() - t_start > 8:
                        break

                stream.stop_stream()
                stream.close()

                dur = len(frames) * self.FRAME_SIZE / self.DEV_RATE
                if not recording or dur < 0.5:
                    continue

                # Resample to 16kHz for ASR
                raw = b''.join(frames)
                audio = np.frombuffer(raw, dtype=np.int16)
                num = int(len(audio) * 16000 / self.DEV_RATE)
                audio = self.resample(audio.astype(np.float32), num).astype(np.int16)

                # ASR
                text = self.asr.generate(audio)
                if text is None or not text.strip():
                    self.get_logger().info('ASR: no text')
                    continue
                self.get_logger().info(f'ASR: {text}')

                # Publish recognized text
                self.text_pub.publish(String(data=f'[ASR] {text}'))

                # LLM
                t0 = time.time()
                reply = chat(text, system=CMD_SYSTEM_PROMPT, max_tokens=40)
                self.get_logger().info(
                    f'LLM ({time.time()-t0:.1f}s): {reply}')
                self.text_pub.publish(String(data=f'[LLM] {reply}'))

                # Extract command
                cmd_match = re.search(r'\[CMD:(\w+)\]', reply)
                if cmd_match:
                    cmd = cmd_match.group(1)
                    self.get_logger().info(f'Command: {cmd}')
                    self.cmd_pub.publish(String(data=cmd))

                # TTS — strip command tags
                clean = re.sub(r'\[CMD:\w+\]', '', reply)[:80].strip()
                clean = clean.replace('"', '').replace("'", '')
                if clean:
                    subprocess.run(
                        ['espeak-ng', clean, '-v', 'zh', '-s', '160',
                         '-w', '/tmp/tts_out.wav'],
                        capture_output=True, timeout=10)
                    subprocess.run(
                        ['paplay', '/tmp/tts_out.wav'],
                        capture_output=True, timeout=10)

            except Exception as e:
                if self._running:
                    self.get_logger().error(f'Voice loop error: {e}')
                time.sleep(1.0)

    def destroy(self):
        self._running = False
        if hasattr(self, '_pa'):
            self._pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
