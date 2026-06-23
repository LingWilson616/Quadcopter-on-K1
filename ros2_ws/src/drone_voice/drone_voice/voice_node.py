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

# PipeWire device names (stable across USB ports)
_PW_SOURCE = 'alsa_input.usb-C-Media_Electronics_Inc._USB_PnP_Sound_Device-00.analog-mono'
_PW_SINK = 'USB_Audio_Device'
DEV_RATE = 48000
FRAME_SIZE = 1536  # 32ms @ 48kHz


def _pick_device(pattern, pactl_cmd, name_hint):
    '''Find a PipeWire device by matching pattern in ``pactl list short`` output.'''
    r = subprocess.run(['pactl', 'list', 'short'] + pactl_cmd,
                       capture_output=True, text=True)
    for line in r.stdout.strip().split('\n'):
        if pattern in line:
            return line.split()[0]
    return None


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        self.cmd_pub = self.create_publisher(String, '/drone/command', 10)
        self.text_pub = self.create_publisher(String, '/drone/voice_text', 10)

        self._running = True
        self._ready = False
        self._parec_proc = None

        self._init_thread = threading.Thread(target=self._init_pipeline, daemon=True)
        self._init_thread.start()

        self._hb_timer = self.create_timer(2.0, self._heartbeat)

    def _heartbeat(self):
        if not self._ready:
            self.get_logger().info('Initializing voice pipeline...')
        else:
            self.destroy_timer(self._hb_timer)

    def _init_pipeline(self):
        try:
            self.get_logger().info('Loading ASR model...')
            self.asr = ASRModel()
            self.get_logger().info('ASR model loaded')

            # --- Audio routing: USB mic for input, USB sound card for output ---
            src_id = _pick_device('USB_PnP_Sound_Device', ['sources'], 'mic')
            if src_id:
                subprocess.run(['pactl', 'set-default-source', src_id],
                               capture_output=True)
                self.get_logger().info(f'Default source: {src_id}')
            else:
                self.get_logger().warn('USB mic not found, using system default')

            sink_id = _pick_device(_PW_SINK, ['sinks'], 'speaker')
            if sink_id:
                subprocess.run(['pactl', 'set-default-sink', sink_id],
                               capture_output=True)
                self.get_logger().info(f'Default sink: {sink_id}')
            else:
                self.get_logger().warn('USB audio device not found')

            # --- VAD ---
            from scipy.signal import resample
            import onnxruntime as ort
            from collections import deque
            self.resample = resample
            self.deque = deque

            vad_path = os.path.expanduser('~/.cache/sensevoice/silero_vad.onnx')
            self.vad_sess = ort.InferenceSession(vad_path, providers=['CPUExecutionProvider'])
            self.get_logger().info('VAD model loaded')

            self._ready = True
            self.get_logger().info('Voice pipeline ready — listening')
            self._voice_loop()

        except Exception as e:
            self.get_logger().error(f'Init failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _voice_loop(self):
        SILENCE_MAX = int(2.5 * DEV_RATE / FRAME_SIZE)  # 2.5s silence → end

        while self._running:
            try:
                self._parec_proc = subprocess.Popen(
                    ['parec', '--format=s16le', '--rate=48000', '--channels=1',
                     '--device=' + _PW_SOURCE],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

                vad_state = np.zeros((2, 1, 128), dtype=np.float32)
                vad_ctx = np.zeros((1, 64), dtype=np.float32)
                vad_sr = np.array(16000, dtype=np.int64)
                pre_buffer = self.deque(maxlen=20)
                frames = []
                recording = False
                silent_count = 0
                t_start = time.time()

                while self._running:
                    raw = self._parec_proc.stdout.read(FRAME_SIZE * 2)
                    if len(raw) < FRAME_SIZE * 2:
                        break

                    frame_np = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                    n_16k = int(len(frame_np) * 16000 / DEV_RATE)
                    frame_16k = self.resample(frame_np, n_16k).astype(np.float32) / 32768.0

                    x = np.concatenate((vad_ctx, frame_16k[np.newaxis, :]), axis=1)
                    vad_ctx = x[:, -64:]
                    prob_arr, vad_state = self.vad_sess.run(
                        None,
                        {'input': x.astype(np.float32), 'state': vad_state, 'sr': vad_sr})
                    prob = float(prob_arr.flat[0])

                    pre_buffer.append(raw)

                    if prob > 0.3 and not recording:
                        recording = True
                        silent_count = 0
                        frames = list(pre_buffer)
                        self.get_logger().info('Speech detected')

                    if recording:
                        frames.append(raw)
                        if prob < 0.25:
                            silent_count += 1
                        else:
                            silent_count = 0
                        if silent_count > SILENCE_MAX:
                            break

                    if time.time() - t_start > 10:
                        break

                self._parec_proc.kill()
                self._parec_proc.wait()
                self._parec_proc = None

                dur = len(frames) * FRAME_SIZE / DEV_RATE
                if not recording or dur < 0.5:
                    continue

                # Resample to 16kHz for ASR
                raw_audio = b''.join(frames)
                audio = np.frombuffer(raw_audio, dtype=np.int16)
                num = int(len(audio) * 16000 / DEV_RATE)
                audio_16k = self.resample(audio.astype(np.float32), num).astype(np.int16)

                # ASR
                text = self.asr.generate(audio_16k)
                if text is None or not text.strip():
                    self.get_logger().info('ASR: no text')
                    continue
                self.get_logger().info(f'ASR: {text}')
                self.text_pub.publish(String(data=f'[ASR] {text}'))

                # LLM
                t0 = time.time()
                reply = chat(text, system=CMD_SYSTEM_PROMPT, max_tokens=40)
                self.get_logger().info(f'LLM ({time.time()-t0:.1f}s): {reply}')
                self.text_pub.publish(String(data=f'[LLM] {reply}'))

                # Extract command
                cmd_match = re.search(r'\[CMD:(\w+)\]', reply)
                if cmd_match:
                    cmd = cmd_match.group(1)
                    self.get_logger().info(f'Command: {cmd}')
                    self.cmd_pub.publish(String(data=cmd))

                # TTS
                clean = re.sub(r'\[CMD:\w+\]', '', reply)[:80].strip()
                clean = clean.replace('"', '').replace("'", '')
                if clean:
                    subprocess.run(
                        ['espeak-ng', clean, '-v', 'zh', '-s', '160', '-a', '15',
                         '-w', '/tmp/tts_out.wav'],
                        capture_output=True, timeout=10)
                    subprocess.run(
                        ['aplay', '-q', '-D', 'plughw:0,0', '/tmp/tts_out.wav'],
                        capture_output=True, timeout=10)

            except Exception as e:
                if self._running:
                    self.get_logger().error(f'Voice loop error: {e}')
                time.sleep(1.0)

    def destroy(self):
        self._running = False
        if self._parec_proc is not None:
            self._parec_proc.kill()
            self._parec_proc.wait()
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
