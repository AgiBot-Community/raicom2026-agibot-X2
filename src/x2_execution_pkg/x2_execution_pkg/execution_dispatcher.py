#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import sherpa_onnx
import threading
import time 
import re

# 导入 TF 库用于坐标转换
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# ================= [双核视觉] =================
from ultralytics import YOLO  # 🌟 引入 YOLO

# ================= [真机接口] =================
from aimdk_msgs.srv import PlayEmoji, SetMcPresetMotion
from aimdk_msgs.msg import AudioPlayback, AudioInfo, AudioData

class ExecutionDispatcherNode(Node):
    def __init__(self):
        super().__init__('execution_dispatcher_node')

        self.get_logger().info("⏳ 正在初始化执行调度中枢(真机部署版 - YOLO/OCR 双核版)...")

        # ================= 1. 视觉AI与图像工具 =================
        
        # 🌟 加载 YOLO 模型 (请确保路径下有针对数字训练的 .pt 模型)
       # 🌟 加载 ONNX 格式的 YOLO 模型
        yolo_model_path = "/home/agi/star_agibot/yolo.onnx"  # 换成你实际下载的 onnx 文件名
        try:
            self.yolo_model = YOLO(yolo_model_path, task='detect') # 显式告诉引擎这是目标检测模型
            self.get_logger().info(f"✅ YOLO 模型已成功加载: {yolo_model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO 模型加载失败！请检查文件是否存在: {e}")
            
        self.cv_bridge = CvBridge()
        self.task3_vision_active = False
        self.task3_history = []
        self.task4_vision_active = False
        self.task4_target_object = "" 

        # ================= 2. 传感器订阅 =================
        self.sub_rgbd_rgb = self.create_subscription(
            Image, '/aima/hal/sensor/rgbd_head_front/rgb_image', self.rgbd_rgb_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, '/aima/hal/sensor/rgbd_head_front/depth_image', self.depth_callback, qos_profile_sensor_data)
        self.sub_rgbd_camera_info = self.create_subscription(
            CameraInfo, '/aima/hal/sensor/rgbd_head_front/camera_info', self.rgbd_camera_info_callback, 10)
        
        self.latest_depth_image = None
        self.camera_intrinsics = None

        # ================= 3. 通信与控制发布 =================
        self.sub_brain = self.create_subscription(String, '/ai_agent/input_json', self.brain_callback, 10)
        self.pub_grasp = self.create_publisher(String, '/competition/grasp_target', 10)
        self.pub_listen = self.create_publisher(Bool, '/ai_agent/listen_control', 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sub_nav_status = self.create_subscription(String, '/race_task/nav_status', self.nav_status_callback, 10)
        
        self.pub_audio = self.create_publisher(AudioPlayback, '/aima/hal/audio/playback', 10)

        # ================= 4. TF 坐标系转换工具 =================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ================= 5. 官方硬件服务客户端 =================
        self.cli_emoji = self.create_client(PlayEmoji, '/aimdk_5Fmsgs/srv/PlayEmoji')
        self.cli_motion = self.create_client(SetMcPresetMotion, '/aimdk_5Fmsgs/srv/SetMcPresetMotion')
        
        self.get_logger().info("⏳ 正在加载 Sherpa-onnx 离线语音合成模型...")
        import os
        model_dir = "/home/agi/star_agibot/vits-zh-hf-fanchen-C" 
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=os.path.join(model_dir, "vits-zh-hf-fanchen-C.onnx"),
                    lexicon=os.path.join(model_dir, "lexicon.txt"),
                    tokens=os.path.join(model_dir, "tokens.txt"),
                ),
                num_threads=2, 
                debug=False
            )
        )
        self.tts_engine = sherpa_onnx.OfflineTts(tts_config)
        self.get_logger().info("✅ 离线语音合成模型加载完毕！")
        self.get_logger().info("✅ 执行调度中枢就绪！等待大脑下发指令...")

    # ================= 麦克风控制 =================
    def set_mic_status(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_listen.publish(msg)
        state_str = "🟢 开启 (等待人类输入)" if enable else "🔴 关闭 (避免噪音干扰)"
        self.get_logger().info(f"🎤 麦克风控制: {state_str}")

    def call_tts(self, text, callback=None):
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
                self._finish_tts(callback)
            except Exception as e:
                self.get_logger().error(f"❌ TTS 流式投递崩溃: {e}")
                if callback: callback(None)

        threading.Thread(target=worker, daemon=True).start()

    def _finish_tts(self, callback=None) -> None:
        self.get_logger().info("✅ 语音播放完毕。")
        if callback is not None:
            callback(None)
        self.set_mic_status(True)
    
    # ================= 硬件控制接口 =================
    def call_emoji(self, emotion_id, callback=None):
        if not self.cli_emoji.wait_for_service(timeout_sec=1.0):
            return
        req = PlayEmoji.Request()
        req.emotion_id = int(emotion_id)
        req.mode = 1       
        req.priority = 100 
        future = self.cli_emoji.call_async(req)
        if callback: future.add_done_callback(callback)

    def call_motion(self, motion_id, area_id, callback=None):
        if not self.cli_motion.wait_for_service(timeout_sec=1.0):
            return
        req = SetMcPresetMotion.Request()
        try:
            req.area.value = int(area_id)
            req.motion.value = int(motion_id)
        except AttributeError:
            req.area = int(area_id)
            req.motion = int(motion_id)
        req.interrupt = True      
        req.play_timestamp = 0    
        future = self.cli_motion.call_async(req)
        if callback: future.add_done_callback(callback)

    # --- 传感器缓存回调 ---
    def rgbd_camera_info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg

    def depth_callback(self, msg):
        if self.task4_vision_active: 
            self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgbd_rgb_callback(self, msg):
        if not self.task3_vision_active and not self.task4_vision_active: 
            return
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.task3_vision_active:
            self.task3_vision_active = False 
            self.process_task3_vision(cv_image)
        if self.task4_vision_active:
            self.task4_vision_active = False 
            self.process_task4_vision(cv_image)

    # ================= 大脑指令解析 =================
    def brain_callback(self, msg):
        try:
            brain_data = json.loads(msg.data)
            intent = brain_data.get('intent_type', 'unknown')
            slots = brain_data.get('slots', {})
            self.get_logger().info(f"🧠 收到大脑指令: {intent}")

            if intent == "task3_time_query":
                self.call_tts(slots.get('reply_text', ''), lambda f: self.set_mic_status(True))
            elif intent == "task3_emoji_control":
                self.call_emoji(slots.get('emotion_id', 90), lambda f: self.set_mic_status(True))
            elif intent == "task3_action_control":
                motion_id, area_id, reply_text = slots.get('motion', 1002), slots.get('area', 1), slots.get('reply_text', '')
                self.call_tts(reply_text, lambda f: self.call_motion(motion_id, area_id, lambda _: self.set_mic_status(True)))
            elif intent == "task3_digit_color_query":
                self.get_logger().info("👀 已开启视觉锁：准备截取任务 3 画面...")
                self.task3_vision_active = True
            elif intent == "task4_wake_service":
                self.call_emoji(slots.get('emotion_id', 120)) 
                self.call_tts(slots.get('reply_text', '今天状态怎么样？'), lambda f: self.set_mic_status(True))
            elif intent == "task4_need_classification":
                self.task4_target_object = slots.get('target_object', 'unknown')
                start_nav = slots.get('start_autonomous_service', False)
                self.set_mic_status(False)
                def on_tts_done(future):
                    if start_nav: self.start_autonomous_navigation()
                self.call_tts(slots.get('reply_text', '好的，这就去办。'), on_tts_done)
            elif intent == "unknown":
                self.call_tts(slots.get('reply_text', '我没有听清楚。'), lambda f: self.set_mic_status(True))
        except Exception as e:
            self.get_logger().error(f"❌ 指令分发与执行编排异常: {e}")

    def start_autonomous_navigation(self):
        self.get_logger().info("🧭 正在向底盘下发目标作业区坐标...")
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  
        goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = -0.26, 1.2, 0.0
        import math; yaw = -1.76
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal_pose.publish(goal_msg)

    def nav_status_callback(self, msg):
        if msg.data == "reached":
            self.get_logger().info(f"👀 [任务4时序] 底盘到达，开启视觉寻找目标 [{self.task4_target_object}]...")
            self.task4_vision_active = True 

# ================= 任务 3：灰度生成完美掩码 + 你的精准裁剪算法 =================
    def process_task3_vision(self, img):
        self.get_logger().info("📸 启动 [灰度寻纸 + 精准单字扣图 + YOLO] 终极方案...")
        try:
            # 1. 深度相机物理矫正 (翻转)
            img = cv2.rotate(img, cv2.ROTATE_180)
            h, w = img.shape[:2]
            cv2.imwrite("/var/tmp/task3_debug_raw.jpg", img)

            # 2. 截取画面中央区域 (ROI)
            roi = img[int(h * 0.15):int(h * 0.90), int(w * 0.20):int(w * 0.80)]
            roi_h, roi_w = roi.shape[:2]
            x_offset, y_offset = int(w * 0.20), int(h * 0.15)

            # 3. 灰度寻找白纸安全区，生成完美掩码 (保留验证过没问题的逻辑)
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            hist = cv2.calcHist([gray_roi], [0], None, [256], [0, 256])
            bright_hist_segment = hist[150:241]
            paper_peak_gray = 150 + int(np.argmax(bright_hist_segment)) if len(bright_hist_segment) > 0 else 200

            lower_gray = max(130, paper_peak_gray - 40)
            _, paper_gray_mask = cv2.threshold(gray_roi, lower_gray, 255, cv2.THRESH_BINARY)
            kernel_paper = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
            paper_gray_mask = cv2.morphologyEx(paper_gray_mask, cv2.MORPH_CLOSE, kernel_paper)

            paper_contours, _ = cv2.findContours(paper_gray_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            safe_paper_mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
            if paper_contours:
                largest_paper = max(paper_contours, key=cv2.contourArea)
                if cv2.contourArea(largest_paper) > 8000:
                    ppx, ppy, ppw, pph = cv2.boundingRect(largest_paper)
                    safe_paper_mask[ppy+15:ppy+pph-15, ppx+15:ppx+ppw-15] = 255
                else:
                    safe_paper_mask[:, :] = 255
            else:
                safe_paper_mask[:, :] = 255

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            color_mask_raw = cv2.inRange(hsv, np.array([0, 45, 35]), np.array([179, 255, 255]))
            
            # 生成你要的完美 mask 并保存
            mask = cv2.bitwise_and(color_mask_raw, safe_paper_mask)
            kernel_digit = np.ones((3,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_digit)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_digit)
            cv2.imwrite("/var/tmp/task3_color_mask.jpg", mask)

            # ======================== 以下完全采用你提供的精准定位与裁剪代码 ========================
            # 4. 寻找连通域，精准锁定数字位置
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                self.get_logger().warn("⚠️ 中央 ROI 中未检测到任何彩色数字连通域！")
                self.set_mic_status(True)
                return

            valid_candidates = []
            img_center_x = roi_w / 2.0
            img_center_y = roi_h / 2.0  # 增加 Y 轴中心
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:
                    continue
                dx, dy, dw, dh = cv2.boundingRect(cnt)
                aspect_ratio = float(dw) / dh
                
                if 0.1 <= aspect_ratio <= 3.0 and 15 <= dh <= roi_h * 0.9:
                    box_center_x = dx + dw / 2.0
                    box_center_y = dy + dh / 2.0
                    # 升级为二维空间真实的直线距离 (欧氏距离)
                    dist_to_center = ((box_center_x - img_center_x)**2 + (box_center_y - img_center_y)**2) ** 0.5
                    valid_candidates.append((cnt, area, dist_to_center, (dx, dy, dw, dh)))

            if not valid_candidates:
                self.get_logger().warn("⚠️ 连通域尺寸过滤后为空！")
                self.set_mic_status(True)
                return

            # 🌟 核心修正：让“距离中心近”成为绝对主导！
            # 距离(x[2])的权重是 1.0，面积(x[1])的权重被压制到极小的 0.005。
            # 这样就算边缘的手面积再大，也绝对争不过位于画面正中央的小个子数字！
            valid_candidates.sort(key=lambda x: x[2] * 1.0 - x[1] * 0.005)
            _, _, _, (dx, dy, dw, dh) = valid_candidates[0]

            # 5. 精准裁剪出独立的数字小图，并利用掩码将背景完美融合为纯白
            digit_crop = roi[dy:dy+dh, dx:dx+dw].copy()
            digit_mask_roi = mask[dy:dy+dh, dx:dx+dw] 
            
            bg_mask = cv2.bitwise_not(digit_mask_roi)
            digit_crop[bg_mask == 255] = [255, 255, 255]
            
            # 四周加上巨大的纯白边框，满足 YOLO 的尺度感
            pad = 200
            digit_padded = cv2.copyMakeBorder(digit_crop, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=[255, 255, 255])
            cv2.imwrite("/var/tmp/task3_single_digit.jpg", digit_padded)
            # ========================================================================================

            # 6. 送交 YOLO 识别
            if not hasattr(self, 'yolo_model'):
                self.get_logger().error("⚠️ YOLO 模型未加载！")
                self.set_mic_status(True)
                return

            results = self.yolo_model(digit_padded, conf=0.25, verbose=False)
            boxes = results[0].boxes

            if len(boxes) == 0:
                self.get_logger().warn("⚠️ 裁剪出的单字图送入 YOLO 后未检出目标！")
                self.set_mic_status(True)
                return

            best_box = max(boxes, key=lambda b: float(b.conf[0].cpu().numpy()))
            best_cls = int(best_box.cls[0].cpu().numpy())
            target_text = results[0].names[best_cls].upper()

            digit_map = {
                '0':'零', 'O':'零', 'Q':'零', 'D':'零',
                '1':'一', 'I':'一', 'L':'一', 'T':'一',
                '2':'二', 'Z':'二',
                '3':'三',
                '4':'四',
                '5':'五', 'S':'五',
                '6':'六', 'G':'六',
                '7':'七',
                '8':'八', 'B':'八',
                '9':'九'
            }
            chinese_target_text = digit_map.get(target_text, target_text)

            # 7. 从数字前景 mask 中精确计算颜色
            digit_hsv = hsv[dy:dy+dh, dx:dx+dw]
            color_ranges = {
                "红": [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [179, 255, 255])],
                "橙": [([11, 100, 100], [25, 255, 255])],
                "黄": [([26, 100, 100], [34, 255, 255])],
                "绿": [([35, 50, 40], [85, 255, 255])],
                "青": [([86, 100, 100], [100, 255, 255])],
                "蓝": [([101, 50, 50], [130, 255, 255])],
                "紫": [([131, 50, 50], [155, 255, 255])]
            }
            
            color_pixels = {}
            for color_name, ranges in color_ranges.items():
                c_mask = np.zeros(digit_hsv.shape[:2], dtype=np.uint8)
                for lower, upper in ranges:
                    c_mask = cv2.bitwise_or(c_mask, cv2.inRange(digit_hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)))
                
                valid_c_mask = cv2.bitwise_and(c_mask, digit_mask_roi)
                color_pixels[color_name] = cv2.countNonZero(valid_c_mask)

            detected_color = max(color_pixels, key=color_pixels.get) if color_pixels else "未知"
            if color_pixels.get(detected_color, 0) < 10: detected_color = "未知"

            self.get_logger().info(f"🏆 精准扣图识别成功 -> 字符: '{chinese_target_text}' (原识别: {target_text}), 颜色: {detected_color}")

            # 8. 画框并语音播报
            orig_x = dx + x_offset
            orig_y = dy + y_offset
            vis_img = img.copy()
            cv2.rectangle(vis_img, (orig_x, orig_y), (orig_x+dw, orig_y+dh), (0, 255, 0), 3)
            cv2.imwrite("/var/tmp/task3_final_result.jpg", vis_img)

            speech_text = f"我看到了，这是{detected_color}色的数字{chinese_target_text}。"
            self.call_tts(speech_text, lambda f: self.set_mic_status(True))

        except Exception as e:
            self.get_logger().error(f"❌ 任务 3 崩溃: {e}")
            self.set_mic_status(True)

    # ================= 任务 4：空间坐标抓取 (保留 OCR) =================
    def process_task4_vision(self, img):
        self.get_logger().info(f"📸 开始寻找抓取目标: {self.task4_target_object}...")
        try:
            img_flipped_for_ocr = cv2.rotate(img, cv2.ROTATE_180)
            pad_size = 100
            padded_img = cv2.copyMakeBorder(img_flipped_for_ocr, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=[255, 255, 255])
            
            ocr_result, _ = self.ocr_engine(padded_img)
            target_box = None
            if ocr_result:
                for res in ocr_result:
                    box_points, text, conf = res
                    if self.task4_target_object in text:
                        target_box = box_points
                        break
                        
            if not target_box:
                self.get_logger().warn(f"⚠️ 画面中未找到标有 [{self.task4_target_object}] 的物品！")
                return
                
            pts = np.array(target_box, dtype=np.int32) - pad_size
            u_flipped, v_flipped = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))
            
            h, w = img.shape[:2]
            u, v = w - 1 - u_flipped, h - 1 - v_flipped

            if self.latest_depth_image is None: return
                
            half_k = 2
            max_y, max_x = self.latest_depth_image.shape
            min_v, max_v = max(0, v - half_k), min(max_y, v + half_k + 1)
            min_u, max_u = max(0, u - half_k), min(max_x, u + half_k + 1)
            roi_depth = self.latest_depth_image[min_v:max_v, min_u:max_u]
            valid_depths = roi_depth[roi_depth > 0]
            if len(valid_depths) == 0: return
            
            z_depth_m = float(np.median(valid_depths)) / 1000.0  
            if self.camera_intrinsics is None: return
                
            fx, cx, fy, cy = self.camera_intrinsics.k[0], self.camera_intrinsics.k[2], self.camera_intrinsics.k[4], self.camera_intrinsics.k[5]
            x_cam = (u - cx) * z_depth_m / fx
            y_cam = (v - cy) * z_depth_m / fy
            z_cam = z_depth_m
            
            try:
                transform = self.tf_buffer.lookup_transform('base_link', self.camera_intrinsics.header.frame_id, rclpy.time.Time())
                cam_pose = tf2_geometry_msgs.PoseStamped()
                cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z = x_cam, y_cam, z_cam
                cam_pose.pose.orientation.w = 1.0
                base_pose = tf2_geometry_msgs.do_transform_pose(cam_pose.pose, transform)
                
                grasp_msg = {"object_name": self.task4_target_object, "target_point": [base_pose.position.x, base_pose.position.y, base_pose.position.z]}
                msg = String()
                msg.data = json.dumps(grasp_msg, ensure_ascii=False)
                self.pub_grasp.publish(msg)
                self.get_logger().info(f"📤 [任务4] 抓取指令发布成功！")
            except Exception as e:
                self.get_logger().error(f"❌ TF 转换失败: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ 任务 4 视觉处理失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ExecutionDispatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()