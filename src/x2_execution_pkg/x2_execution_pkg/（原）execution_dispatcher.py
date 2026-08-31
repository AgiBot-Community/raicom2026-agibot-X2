#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool  # [新增] 导入 Bool 类型用于麦克风控制
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# 导入 TF 库用于坐标转换
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from rapidocr_onnxruntime import RapidOCR

class ExecutionDispatcherNode(Node):
    def __init__(self):
        super().__init__('execution_dispatcher_node')

        self.get_logger().info("⏳ 正在初始化执行调度中枢...")

        # ================= 1. 视觉AI与图像工具 =================
        self.ocr_engine = RapidOCR(use_cls=False)
        self.cv_bridge = CvBridge()
        
        self.task3_vision_active = False
        self.task4_vision_active = False
        self.task4_target_object = "" 

        # ================= 2. 传感器订阅 (视觉输入) =================
        self.sub_rgb = self.create_subscription(
            Image, '/aima/hal/sensor/rgbd_head_front/rgb_image', self.rgb_callback, qos_profile_sensor_data)
        
        self.sub_depth = self.create_subscription(
            Image, '/aima/hal/sensor/rgbd_head_front/depth_image', self.depth_callback, qos_profile_sensor_data)
        
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/aima/hal/sensor/rgbd_head_front/camera_info', self.camera_info_callback, 10)
        
        self.latest_depth_image = None
        self.camera_intrinsics = None

        # ================= 3. 大脑与运控通信 =================
        self.sub_brain = self.create_subscription(
            String, '/ai_agent/input_json', self.brain_callback, 10)
        
        self.pub_grasp = self.create_publisher(
            String, '/competition/grasp_target', 10)
            
        # [修改点 2]: 只增加麦克风控制的话题发布者
        self.pub_listen = self.create_publisher(
            Bool, '/ai_agent/listen_control', 10)

        # ================= 4. TF 坐标系转换工具 =================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ================= 5. 官方服务客户端 (继续保持注释，用于本地测试) =================
        # self.cli_tts = self.create_client(PlayTts, '/aimdk_msgs/srv/PlayTts')
        # self.cli_emoji = self.create_client(PlayEmoji, '/aimdk_msgs/srv/PlayEmoji')
        # self.cli_motion = self.create_client(SetMcPresetMotion, '/aimdk_msgs/srv/SetMcPresetMotion')

        self.get_logger().info("✅ 执行调度中枢就绪！等待大脑下发指令...")

    # ================= [新增] 麦克风控制专用函数 =================
    def set_mic_status(self, enable: bool):
        """控制底层语音节点的麦克风开关"""
        msg = Bool()
        msg.data = enable
        self.pub_listen.publish(msg)
        state_str = "🟢 开启 (等待人类输入)" if enable else "🔴 关闭 (避免噪音干扰)"
        self.get_logger().info(f"🎤 麦克风控制: {state_str}")

    # --- 以下是传感器数据的被动缓存回调 ---
    def depth_callback(self, msg):
        if self.task4_vision_active: 
            self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg
            self.get_logger().info("📷 已成功锁存相机内参矩阵！")

    def rgb_callback(self, msg):
        if not self.task3_vision_active and not self.task4_vision_active:
            return

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.task3_vision_active:
            self.task3_vision_active = False 
            self.process_task3_vision(cv_image)
            
        elif self.task4_vision_active:
            self.task4_vision_active = False 
            self.process_task4_vision(cv_image)

    # --- 大脑指令解析与分发 ---
    def brain_callback(self, msg):
        try:
            brain_data = json.loads(msg.data)
            intent = brain_data.get('intent_type', 'unknown')
            slots = brain_data.get('slots', {})

            self.get_logger().info(f"🧠 收到大脑指令: {intent}")

            # ================= [任务 3 逻辑分流] =================
            if intent == "task3_time_query":
                reply_text = slots.get('reply_text', '')
                self.get_logger().info(f"🗣️ [模拟 TTS 播报]: {reply_text}")
                self.set_mic_status(True) # 播报完重新听

            elif intent == "task3_emoji_control":
                emoji = slots.get('emoji', 'happy')
                self.get_logger().info(f"😊 [模拟 表情切换]: {emoji}")
                self.set_mic_status(True) # 表情做完重新听

            elif intent == "task3_action_control":
                action = slots.get('action', 'wave_left')
                reply_text = slots.get('reply_text', '')
                self.get_logger().info(f"🗣️ [模拟 TTS 播报]: {reply_text}")
                self.get_logger().info(f"🤖 [模拟 肢体动作]: {action}")
                self.set_mic_status(True) # 动作做完重新听

            elif intent == "task3_digit_color_query":
                self.get_logger().info("👀 已开启视觉锁：准备截取任务 3 的单帧画面...")
                self.task3_vision_active = True
                # 注意：这里不控制麦克风，麦克风在 process_task3_vision 处理完视觉后再打开

            # ================= [任务 4 逻辑分流 (核心时序更改)] =================
            elif intent == "task4_wake_service":
                reply_text = slots.get('reply_text', '今天状态怎么样？')
                self.get_logger().info(f"😊 [模拟 表情切换]: 关切/聆听")
                self.get_logger().info(f"🗣️ [模拟 TTS 播报]: {reply_text}")
                self.set_mic_status(True) # 唤醒并询问后，必须开麦听需求

            elif intent == "task4_need_classification":
                # [修改点 1]: 时序控制第一步 - 提取目标并说话，关闭麦克风
                self.task4_target_object = slots.get('target_object', 'unknown')
                reply_text = slots.get('reply_text', '好的，这就去办。')
                
                self.get_logger().info(f"🗣️ [模拟 TTS 播报]: {reply_text}")
                self.set_mic_status(False) # 机器人在跑腿和抓取，必须关麦避免听见环境噪音
                
                # 时序控制第二步 - 启动模拟导航 (3秒后触发视觉)
                self.get_logger().info("🧭 [任务4时序] 第一阶段完成：已播报回复。")
                self.get_logger().info("🧭 [任务4时序] 第二阶段开始：启动自主导航前往作业区...")
                # 开启一个 3 秒的一次性定时器，模拟机器人在物理空间移动的时间
                self.nav_timer = self.create_timer(3.0, self.mock_navigation_done_callback)

            else:
                self.get_logger().warn(f"⚠️ 收到未知的意图类型: {intent}")

        except json.JSONDecodeError:
            self.get_logger().error(f"❌ 解析大脑 JSON 失败，格式错误: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"❌ 指令分发时发生异常: {e}")

    # ================= [新增] 模拟导航完成的回调 =================
    def mock_navigation_done_callback(self):
        """模拟机器人底盘到达桌前，开始第三阶段：开启视觉"""
        self.nav_timer.cancel() # 取消定时器，避免重复触发
        self.get_logger().info("🛑 [任务4时序] 第二阶段完成：导航到达作业区，底盘已停稳！")
        self.get_logger().info(f"👀 [任务4时序] 第三阶段开始：开启视觉锁，寻找目标物品 [{self.task4_target_object}]...")
        self.task4_vision_active = True # 正式唤醒视觉去抓图

    # ================= 视觉处理逻辑 (恢复严谨日志版) =================
    def process_task3_vision(self, img):
        self.get_logger().info("📸 成功截取单帧画面，开始执行 RapidOCR 与颜色识别...")
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, enhanced_mask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY_INV) 
            pad_size = 100
            padded_img = cv2.copyMakeBorder(img, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=[255, 255, 255])
            
            ocr_result, _ = self.ocr_engine(padded_img)
            
            if not ocr_result:
                self.get_logger().warn("⚠️ 画面中未识别到任何文字或数字！")
                self.set_mic_status(True) # 就算识别失败也要开麦，防止流程卡死
                return
                
            target_box, target_text = None, ""
            for res in ocr_result:
                box_points, text, conf = res
                
                # [恢复] 1. 打印 OCR 扫描到的所有可疑文本与置信度
                self.get_logger().info(f"🔍 [Debug] OCR 发现可疑文本: '{text}', 置信度: {conf}")
                
                text = text.upper().replace('S', '2').replace('O', '0').replace('Z', '2')
                if any(char.isdigit() for char in text): 
                    target_box = box_points
                    target_text = ''.join(filter(str.isdigit, text))
                    break
                    
            if not target_box:
                self.get_logger().warn("⚠️ 画面中未找到包含数字的区域！")
                self.set_mic_status(True) # 识别失败，开麦重新听
                return

            pts = np.array(target_box, dtype=np.int32) - pad_size
            x1, x2 = int(np.min(pts[:, 0])), int(np.max(pts[:, 0]))
            y1, y2 = int(np.min(pts[:, 1])), int(np.max(pts[:, 1]))
            
            # [恢复] 2. 打印还原后的目标边框坐标
            self.get_logger().info(f"🎯 提取目标数字: {target_text}, 边框坐标: [{x1}, {y1}, {x2}, {y2}]")
            
            roi = img[max(0, y1):min(img.shape[0], y2), max(0, x1):min(img.shape[1], x2)]
            
            if roi.size == 0:
                self.get_logger().warn("⚠️ ROI 裁剪失败！")
                self.set_mic_status(True)
                return

            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            color_ranges = {
                "红": [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [179, 255, 255])],
                "蓝": [([85, 50, 50], [130, 255, 255])],
                "绿": [([35, 100, 100], [85, 255, 255])],
                "黄": [([15, 100, 100], [34, 255, 255])],
                "黑": [([0, 0, 0], [180, 255, 50])]
            }
            
            color_pixels = {}
            for color_name, ranges in color_ranges.items():
                mask = np.zeros(hsv_roi.shape[:2], dtype=np.uint8)
                for lower, upper in ranges:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv_roi, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)))
                color_pixels[color_name] = cv2.countNonZero(mask)

            detected_color = max(color_pixels, key=color_pixels.get) if color_pixels else "未知"
            if color_pixels.get(detected_color, 0) < 10: detected_color = "未知"

            # [恢复] 3. 打印底层的颜色像素得分
            self.get_logger().info(f"🎨 颜色判定结果: {detected_color} (详细得分: {color_pixels})")

            speech_text = f"我看到了，这是{detected_color}色的数字{target_text}。"
            self.get_logger().info(f"🗣️ [模拟 TTS 播报]: {speech_text}")
            
            # 看图说话的最终环节，开启麦克风听取下一个任务
            self.set_mic_status(True)
            
        except Exception as e:
            self.get_logger().error(f"❌ 任务 3 视觉处理失败: {e}")
            self.set_mic_status(True)
            
    def process_task4_vision(self, img):
        self.get_logger().info(f"📸 开始寻找抓取目标: {self.task4_target_object}...")
        try:
            pad_size = 100
            padded_img = cv2.copyMakeBorder(img, pad_size, pad_size, pad_size, pad_size, cv2.BORDER_CONSTANT, value=[255, 255, 255])
            
            ocr_result, _ = self.ocr_engine(padded_img)
            target_box = None
            if ocr_result:
                for res in ocr_result:
                    box_points, text, conf = res
                    self.get_logger().info(f"🔍 [Debug] 任务 4 发现可疑文本: '{text}'")
                    if self.task4_target_object in text:
                        target_box = box_points
                        break
                        
            if not target_box:
                self.get_logger().warn(f"⚠️ 画面中未找到标有 [{self.task4_target_object}] 的物品！")
                return
                
            pts = np.array(target_box, dtype=np.int32) - pad_size
            u, v = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))
            self.get_logger().info(f"🎯 找到物品 2D 中心像素点: (u={u}, v={v})")

            if self.latest_depth_image is None:
                self.get_logger().warn("⚠️ 尚未接收到深度图数据，无法测距！")
                return
                
            z_depth_mm = self.latest_depth_image[v, u]
            if z_depth_mm == 0 or np.isnan(z_depth_mm):
                self.get_logger().warn("⚠️ 物品中心点深度值为无效值 (反光或超出量程)！")
                return
                
            z_depth_m = float(z_depth_mm) / 1000.0  
            
            if self.camera_intrinsics is None:
                self.get_logger().warn("⚠️ 尚未接收到相机内参矩阵，无法计算 3D 坐标！")
                return
                
            fx, cx, fy, cy = self.camera_intrinsics.k[0], self.camera_intrinsics.k[2], self.camera_intrinsics.k[4], self.camera_intrinsics.k[5]
            x_cam = (u - cx) * z_depth_m / fx
            y_cam = (v - cy) * z_depth_m / fy
            z_cam = z_depth_m
            
            self.get_logger().info(f"📏 相机 3D 坐标系位置: x={x_cam:.3f}, y={y_cam:.3f}, z={z_cam:.3f}")

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
                
                self.get_logger().info(f"📤 [任务4时序] 第四阶段完成：抓取坐标已发布给夹爪！自主服务阶段闭环！")
                
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