#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from aimdk_msgs.msg import AudioPlayback, AudioInfo, AudioData
import sherpa_onnx
import threading
import time
import re
import numpy as np
import os
import json

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.get_logger().info("🎤 语音中枢节点启动中...")

        self.pub_listen = self.create_publisher(Bool, '/ai_agent/listen_control', 10)
        self.pub_audio = self.create_publisher(AudioPlayback, '/aima/hal/audio/playback', 10)
        self.sub_speak = self.create_subscription(String, '/system/voice_cmd', self.speak_callback, 10)

        model_dir = "/home/agi/star_agibot/vits-zh-hf-fanchen-C" 
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=os.path.join(model_dir, "vits-zh-hf-fanchen-C.onnx"),
                    lexicon=os.path.join(model_dir, "lexicon.txt"),
                    tokens=os.path.join(model_dir, "tokens.txt"),
                ),
                num_threads=2, debug=False
            )
        )
        self.tts_engine = sherpa_onnx.OfflineTts(tts_config)
        self.get_logger().info("✅ 语音中枢就绪！")

    def set_mic_status(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_listen.publish(msg)
        self.get_logger().info(f"麦克风控制: {'🟢 开启' if enable else '🔴 关闭'}")

    def speak_callback(self, msg):
        # 🌟 尝试解析 JSON 以获取控制标志
        try:
            cmd_data = json.loads(msg.data)
            text = cmd_data.get("text", "")
            auto_open_mic = cmd_data.get("auto_open_mic", True)
        except json.JSONDecodeError:
            # 兼容旧版本纯文本
            text = msg.data
            auto_open_mic = True

        # ❌ (已删除) self.set_mic_status(False) 
        # 💡 既然大脑已经主动关过了，这里坚决不再多此一举，直接进入生成播报流程
        self.call_tts(text, auto_open_mic)        

    def call_tts(self, text, auto_open_mic):
        def convert_num_to_zh(match):
            num_str = match.group()
            num = int(num_str)
            chars = "零一二三四五六七八九"
            if num < 10:
                return chars[num]
            elif 10 <= num < 20: 
                return "十" + (chars[num % 10] if num % 10 != 0 else "")
            elif 20 <= num < 100: 
                return chars[num // 10] + "十" + (chars[num % 10] if num % 10 != 0 else "")
            else:
                return "".join(chars[int(d)] for d in num_str)

        safe_text = re.sub(r'\d+', convert_num_to_zh, str(text))
        self.get_logger().info(f"🗣️ [TTS生成中]: 原文='{text}' -> 实际播报='{safe_text}'")

        def worker():
            try:
                audio = self.tts_engine.generate(safe_text, sid=0, speed=1.0)
                samples = np.asarray(audio.samples, dtype=np.float32).squeeze()
                original_sr = int(audio.sample_rate)
                target_sr = 16000
                duration_sec = len(samples) / float(original_sr)
                if original_sr != target_sr:
                    target_length = int(duration_sec * target_sr)
                    x_old = np.linspace(0, duration_sec, len(samples))
                    x_new = np.linspace(0, duration_sec, target_length)
                    samples = np.interp(x_new, x_old, samples)
                duration_sec = len(samples) / float(target_sr)
                audio_int16 = (samples * 32767.0).astype(np.int16)
                audio_bytes = audio_int16.tobytes()

                msg = AudioPlayback()
                msg.stamps = self.get_clock().now().to_msg()
                info = AudioInfo()
                info.channels = 1
                info.sample_rate = target_sr
                info.size = len(audio_bytes)
                info.sample_format = "S16_LE"
                info.coding_format = "pcm"
                msg.info = info
                
                adata = AudioData()
                adata.data = list(audio_bytes)
                msg.data = adata
                msg.pkg_name = "agent" 
                msg.token_id = str(time.time())
                
                self.pub_audio.publish(msg)
                time.sleep(duration_sec + 0.3) 
                
                self.get_logger().info("✅ 语音播放完毕。")
                
                # 🌟 根据标志决定是否重新开启麦克风
                if auto_open_mic:
                    self.set_mic_status(True)
                else:
                    self.get_logger().info("🔇 按大脑指令，保持麦克风关闭状态！")
                    
            except Exception as e:
                self.get_logger().error(f"❌ TTS 流式投递崩溃: {e}")
                self.set_mic_status(True)

        threading.Thread(target=worker, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()