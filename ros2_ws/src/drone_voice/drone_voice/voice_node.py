#!/usr/bin/env python3
'''K1 语音交互 ROS2 节点 — VAD→ASR→LLM→MatchTTS 管道, 发布命令到 /drone/command'''
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import sys, os, time, subprocess, threading, re, wave
import numpy as np
import pypinyin
import requests

os.environ['JACK_NO_START_SERVER'] = '1'

sys.path.insert(0, os.path.expanduser('~/spacemit-demo/examples/NLP'))
from spacemit_asr import ASRModel

CMD_SYSTEM_PROMPT = '''你是K1无人机语音助手，只能用中文回答，不超过25字。禁止使用英文。
仅在用户明确要求执行以下动作时，才在回复末尾加上对应标签：
  解锁飞控 → 回复末尾加 [CMD:ARM]
  起飞 → 回复末尾加 [CMD:TAKEOFF]
  降落 → 回复末尾加 [CMD:LAND]
  返航 → 回复末尾加 [CMD:RTL]
如果用户没有要求飞行操作，正常回答问题，不要加任何标签。'''

_PW_SOURCE = 'alsa_input.usb-C-Media_Electronics_Inc._USB_PnP_Sound_Device-00.analog-mono'
_PW_SINK = 'USB_Audio_Device'
DEV_RATE = 48000
FRAME_SIZE = 1536


def _pick_device(pattern, pactl_cmd, name_hint):
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

            src_id = _pick_device('USB_PnP_Sound_Device', ['sources'], 'mic')
            if src_id:
                subprocess.run(['pactl', 'set-default-source', src_id], capture_output=True)
                self.get_logger().info(f'Default source: {src_id}')
            else:
                self.get_logger().warn('USB mic not found')

            sink_id = _pick_device(_PW_SINK, ['sinks'], 'speaker')
            if sink_id:
                subprocess.run(['pactl', 'set-default-sink', sink_id], capture_output=True)
                self.get_logger().info(f'Default sink: {sink_id}')

            from scipy.signal import resample
            import onnxruntime as ort
            from collections import deque
            self.resample = resample
            self.deque = deque
            vad_path = os.path.expanduser('~/.cache/sensevoice/silero_vad.onnx')
            self.vad_sess = ort.InferenceSession(vad_path, providers=['CPUExecutionProvider'])
            self.get_logger().info('VAD model loaded')

            self._ready = True
            self.get_logger().info('Voice pipeline ready')
            threading.Thread(target=self._init_tts, daemon=True).start()
            self._voice_loop()

        except Exception as e:
            self.get_logger().error(f'Init failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _voice_loop(self):
        SILENCE_MAX = int(2.5 * DEV_RATE / FRAME_SIZE)

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
                        None, {'input': x.astype(np.float32), 'state': vad_state, 'sr': vad_sr})
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

                raw_audio = b''.join(frames)
                audio = np.frombuffer(raw_audio, dtype=np.int16)
                num = int(len(audio) * 16000 / DEV_RATE)
                audio_16k = self.resample(audio.astype(np.float32), num).astype(np.int16)

                text = self.asr.generate(audio_16k)
                if text is None or not text.strip():
                    self.get_logger().info('ASR: no text')
                    continue
                self.get_logger().info(f'ASR: {text}')
                self.text_pub.publish(String(data=f'[ASR] {text}'))

                t0 = time.time()
                reply = self._llm_chat(text, system=CMD_SYSTEM_PROMPT, max_tokens=40)
                self.get_logger().info(f'LLM ({time.time()-t0:.1f}s): {reply}')
                self.text_pub.publish(String(data=f'[LLM] {reply}'))

                # Extract command
                cmd_match = re.search(r'\[CMD:(\w+)\]', reply)
                if cmd_match:
                    cmd = cmd_match.group(1)
                    self.get_logger().info(f'Command: {cmd}')
                    self.cmd_pub.publish(String(data=cmd))

            except Exception as e:
                if self._running:
                    self.get_logger().error(f'Voice loop error: {e}')
                time.sleep(1.0)

    def _llm_chat(self, user_text, system=None, max_tokens=40):
        '''Non-streaming chat, synthesize full reply at once.'''
        import json as _json
        try:
            r = requests.post(
                'http://127.0.0.1:8081/v1/chat/completions',
                json={
                    'messages': [
                        {'role': 'system', 'content': system or CMD_SYSTEM_PROMPT},
                        {'role': 'user', 'content': user_text},
                    ],
                    'max_tokens': max_tokens,
                    'temperature': 0.7,
                    'stream': False,
                },
                timeout=30)
            data = r.json()
            reply = data['choices'][0]['message']['content'].strip()
            # Clean and speak
            clean = re.sub(r'\[CMD:\w+\]', '', reply)[:80].strip()
            clean = clean.replace('"', '').replace("'", '')
            clean = re.sub(r'[^一-鿿㐀-䶿a-zA-Z0-9\s,.!?，。！？]', '', clean)
            if clean:
                self._tts_synthesize(clean)
            return reply
        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return ''

    def _init_tts(self):
        try:
            import onnxruntime as ort
            base = os.path.expanduser('~/.cache/matcha-icefall-zh-baker')

            self._tts_t2i = {}
            with open(os.path.join(base, 'tokens.txt')) as f:
                for ln in f:
                    p = ln.strip().split()
                    if len(p) == 2: self._tts_t2i[p[0]] = int(p[1])
            self._tts_blank = self._tts_t2i.get('_', 1)

            self._tts_matcha = ort.InferenceSession(
                os.path.join(base, 'model-steps-3.q.onnx'),
                providers=['CPUExecutionProvider'])
            self._tts_vocos = ort.InferenceSession(
                os.path.expanduser('~/.cache/vocos_22k.q.onnx'),
                providers=['CPUExecutionProvider'])
            self.get_logger().info('TTS models loaded')
        except Exception as e:
            self.get_logger().error(f'TTS init failed: {e}')

    def _tts_synthesize(self, text):
        if not hasattr(self, '_tts_matcha') or not text.strip():
            return

        # Convert digits to Chinese number words
        cn_nums = ['零','一','二','三','四','五','六','七','八','九']
        text = re.sub(r'(\d+)年', lambda m: ''.join(cn_nums[int(d)] for d in m.group(1)) + '年', text)
        text = re.sub(r'(\d+)月', lambda m: ''.join(cn_nums[int(d)] for d in m.group(1)) + '月', text)
        text = re.sub(r'(\d+)日', lambda m: ''.join(cn_nums[int(d)] for d in m.group(1)) + '日', text)
        text = re.sub(r'(\d+)', lambda m: ' '.join(cn_nums[int(d)] for d in m.group(1)), text)

        ids = [self._tts_blank]
        for py in pypinyin.lazy_pinyin(text, style=pypinyin.Style.TONE3,
                                        neutral_tone_with_five=True):
            if py in self._tts_t2i:
                ids.append(self._tts_t2i[py])
            else:
                nt = py.rstrip('012345')
                if nt in self._tts_t2i:
                    ids.append(self._tts_t2i[nt])
            ids.append(self._tts_blank)

        x = np.array([ids], dtype=np.int64)
        xl = np.array([len(ids)], dtype=np.int64)
        ns = np.array([0.667], dtype=np.float32)
        ls = np.array([1.0], dtype=np.float32)

        mel = self._tts_matcha.run(None, {
            'x': x, 'x_length': xl, 'noise_scale': ns, 'length_scale': ls})[0]
        out = self._tts_vocos.run(None, {'mels': mel})
        mag = out[0][0]

        n_fft, hop = 1024, 256
        nf, nt = mag.shape
        win = np.hanning(n_fft)
        alen = (nt - 1) * hop + n_fft
        ang = np.exp(1j * 2 * np.pi * np.random.rand(nf, nt))
        for _ in range(20):
            spec = mag * ang
            sig = np.zeros(alen)
            wsum = np.zeros(alen)
            for i in range(nt):
                frm = np.fft.irfft(spec[:, i], n=n_fft)
                s = i * hop
                sig[s:s+n_fft] += frm * win
                wsum[s:s+n_fft] += win * win
            sig /= (wsum + 1e-8)
            for i in range(nt):
                s = i * hop
                spec[:, i] = np.fft.rfft(sig[s:s+n_fft] * win, n=n_fft)
            ang = spec / (np.abs(spec) + 1e-8)
        audio = sig.astype(np.float32)

        audio *= 0.3 / max(abs(audio))
        a16 = (audio * 32767).clip(-32768, 32767).astype(np.int16)

        with wave.open('/tmp/tts_out.wav', 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(22050)
            w.writeframes(a16.tobytes())

        subprocess.run(
            ['aplay', '-q', '-D', 'plughw:0,0', '/tmp/tts_out.wav'], timeout=10)

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
