#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info("👁️ 视觉中枢启动中...")

        self.cv_bridge = CvBridge()
        
        # 🌟 1. 加载任务 3 数字识别模型
        yolo_digit_path = "/home/agi/star_agibot/yolo.onnx"
        try:
            self.yolo_digit_model = YOLO(yolo_digit_path, task='detect')
            self.get_logger().info("✅ 任务3 数字识别模型加载成功！")
        except Exception as e:
            self.get_logger().error(f"❌ 任务3 YOLO 模型加载失败: {e}")

        # 🌟 2. 加载任务 4 (Plan B) 导出的 YOLOE-ONNX 模型
        yoloe_task4_path = "/home/agi/star_agibot/yoloe_task4.onnx"
        try:
            self.yolo_task4_model = YOLO(yoloe_task4_path, task='segment')
            self.get_logger().info("✅ 任务4 Plan B 大模型加载成功！")
        except Exception as e:
            self.get_logger().error(f"❌ 任务4 Plan B 模型加载失败: {e}")

        # 🌟 2.5 加载任务 4 (Plan A) 专属特训模型
        yolov8s_task4_path = "/home/agi/star_agibot/best.onnx" # ⚠️ 请替换为你的 52 张图训练的 onnx 真实路径
        try:
            self.yolo_task4_planA_model = YOLO(yolov8s_task4_path, task='segment')
            self.get_logger().info("✅ 任务4 Plan A 特训模型加载成功！")
        except Exception as e:
            self.get_logger().error(f"❌ 任务4 Plan A 模型加载失败: {e}")

        # 🌟 新增：扫描计数器与模型切换标志
        self.task4_scan_count = 0
        self.task4_use_plan_a = False

        # 🌟 3. ONNX 提示词到归一化类别的映射字典 (完美适配 Plan B)
        self.class_map = {
            # --- 药盒映射 ---
            "medicine box": "medicine_box",
            "pharmaceutical box": "medicine_box",
            "tall white medicine bottle with an orange cap": "medicine_box",
            
            # --- 纸杯映射 ---
            "paper cup": "paper_cup",
            "disposable paper cup": "paper_cup",
            "empty white paper cup": "paper_cup",
            "white disposable cup": "paper_cup",
            
            # --- 面包映射 ---
            "small bread": "bread",
            "bread roll": "bread",
            "bread bun": "bread",
            "bread wrapped in a transparent plastic bag": "bread",
            "a clear plastic bag filled with food": "bread",
            "yellowish bread inside a transparent bag": "bread", # 修复了此处漏掉的逗号
            "a bulky white block with a plastic knot on top": "bread"
        }
        
        # 🌟 4. 大模型意图到内部归一化类别的映射字典（全面兼容中文、英文名、英文缩写）
        self.brain_intent_to_target = {
            "药盒": "medicine_box",
            "medicine_box": "medicine_box",
            "medicine box": "medicine_box",
            
            "纸杯": "paper_cup",
            "paper_cup": "paper_cup",
            "cup": "paper_cup",         # 👈 兼容大脑发来的 "cup"
            "paper cup": "paper_cup",
            
            "小面包": "bread",
            "bread": "bread",
            "small bread": "bread"
        }

        # TF2 缓存
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_task = None
        self.task4_target_object = ""
        self.latest_depth_image = None
        self.camera_intrinsics = None

        self.sub_rgb = self.create_subscription(Image, '/aima/hal/sensor/rgbd_head_front/rgb_image', self.rgb_cb, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(Image, '/aima/hal/sensor/rgbd_head_front/depth_image', self.depth_cb, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(CameraInfo, '/aima/hal/sensor/rgbd_head_front/depth_camera_info', self.info_cb, qos_profile_sensor_data)

        self.sub_cmd = self.create_subscription(String, '/system/vision_cmd', self.cmd_cb, 10)
        self.pub_result = self.create_publisher(String, '/system/vision_result', 10)
        self.pub_grasp = self.create_publisher(String, '/competition/grasp_target', 10)
        
        self.get_logger().info("✅ 视觉中枢就绪！")

    def info_cb(self, msg): self.camera_intrinsics = msg
    def depth_cb(self, msg): 
        if self.current_task == "4": self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def cmd_cb(self, msg):
        cmd = json.loads(msg.data)
        self.current_task = cmd.get("task")
        self.task4_target_object = cmd.get("target", "")
        
        # 🌟 每次收到新指令时，重置扫描计数和模型标志
        if self.current_task == "4":
            self.task4_scan_count = 0
            self.task4_use_plan_a = False
            
        self.get_logger().info(f"👀 收到视觉触发指令: 任务 {self.current_task}, 目标物品: {self.task4_target_object}")

    def rgb_cb(self, msg):
        if not self.current_task: return
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.current_task == "3":
            result_text = self.process_task3_vision(cv_image) 
            if result_text:
                self.current_task = None 
                self.pub_result.publish(String(data=json.dumps({"task": "3", "result": result_text})))
                
        elif self.current_task == "4":
            self.process_task4_vision(cv_image)

# ================= 任务 3：灰度生成完美掩码 + 精准单字扣图 =================
    def process_task3_vision(self, img):
        try:
            img = cv2.rotate(img, cv2.ROTATE_180)
            h, w = img.shape[:2]
            cv2.imwrite("/var/tmp/task3_debug_raw.jpg", img)

            roi = img[int(h * 0.15):int(h * 0.90), int(w * 0.20):int(w * 0.80)]
            roi_h, roi_w = roi.shape[:2]
            x_offset, y_offset = int(w * 0.20), int(h * 0.15)

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
            
            mask = cv2.bitwise_and(color_mask_raw, safe_paper_mask)
            kernel_digit = np.ones((3,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_digit)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_digit)
            cv2.imwrite("/var/tmp/task3_color_mask.jpg", mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                self.get_logger().warn("⚠️ 中央 ROI 中未检测到任何彩色数字连通域！", throttle_duration_sec=2.0)
                return None

            valid_candidates = []
            img_center_x, img_center_y = roi_w / 2.0, roi_h / 2.0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100: continue
                dx, dy, dw, dh = cv2.boundingRect(cnt)
                aspect_ratio = float(dw) / dh
                if 0.1 <= aspect_ratio <= 3.0 and 15 <= dh <= roi_h * 0.9:
                    box_center_x, box_center_y = dx + dw / 2.0, dy + dh / 2.0
                    dist_to_center = ((box_center_x - img_center_x)**2 + (box_center_y - img_center_y)**2) ** 0.5
                    valid_candidates.append((cnt, area, dist_to_center, (dx, dy, dw, dh)))

            if not valid_candidates:
                self.get_logger().warn("⚠️ 连通域尺寸过滤后为空！", throttle_duration_sec=2.0)
                return None

            valid_candidates.sort(key=lambda x: x[2] * 1.0 - x[1] * 0.005)
            _, _, _, (dx, dy, dw, dh) = valid_candidates[0]

            digit_crop = roi[dy:dy+dh, dx:dx+dw].copy()
            digit_mask_roi = mask[dy:dy+dh, dx:dx+dw] 
            
            bg_mask = cv2.bitwise_not(digit_mask_roi)
            digit_crop[bg_mask == 255] = [255, 255, 255]
            
            # ... 前面代码保持不变 ...
            pad = 200
            digit_padded = cv2.copyMakeBorder(digit_crop, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=[255, 255, 255])
            cv2.imwrite("/var/tmp/task3_single_digit.jpg", digit_padded)

            # ================= 🌟 新增：Otsu Otsu 极速二值化预处理 =================
            # 1. 转换为灰度图
            gray_padded = cv2.cvtColor(digit_padded, cv2.COLOR_BGR2GRAY)
            # 2. Otsu 算法自动寻找最佳阈值，生成对比度极高的黑白图（白底黑字）
            _, binary_padded = cv2.threshold(gray_padded, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            # 3. 将单通道黑白图转回 3 通道 BGR 格式（保证符合 YOLO 模型 3 通道输入要求）
            yolo_input = cv2.cvtColor(binary_padded, cv2.COLOR_GRAY2BGR)
            # 调试用：可保存看二值化后的效果
            cv2.imwrite("/var/tmp/task3_otsu_digit.jpg", yolo_input)
            # ======================================================================

            if not hasattr(self, 'yolo_digit_model'):
                self.get_logger().error("⚠️ 任务3 YOLO 数字模型未加载！", throttle_duration_sec=2.0)
                return None

            # ⚠️ 注意：这里将原来输入的 digit_padded 改为二值化后的 yolo_input
            results = self.yolo_digit_model(yolo_input, conf=0.25, verbose=False)
            boxes = results[0].boxes
            # ... 后面代码保持不变 ...

            if len(boxes) == 0:
                self.get_logger().warn("⚠️ 裁剪出的单字图送入 YOLO 后未检出目标！等待下一帧...", throttle_duration_sec=2.0)
                return None

            best_box = max(boxes, key=lambda b: float(b.conf[0].cpu().numpy()))
            best_cls = int(best_box.cls[0].cpu().numpy())
            target_text = results[0].names[best_cls].upper()

            digit_map = {'0':'零', 'O':'零', 'Q':'零', 'D':'零', '1':'一', 'I':'一', 'L':'一', 'T':'一', '2':'二', 'Z':'二', '3':'三', '4':'四', '5':'五', 'S':'五', '6':'六', 'G':'六', '7':'七', '8':'八', 'B':'八', '9':'九'}
            chinese_target_text = digit_map.get(target_text, target_text)

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

            orig_x, orig_y = dx + x_offset, dy + y_offset
            vis_img = img.copy()
            cv2.rectangle(vis_img, (orig_x, orig_y), (orig_x+dw, orig_y+dh), (0, 255, 0), 3)
            cv2.imwrite("/var/tmp/task3_final_result.jpg", vis_img)

            return f"我看到了，这是{detected_color}色的数字{chinese_target_text}。"

        except Exception as e:
            self.get_logger().error(f"❌ 任务 3 崩溃: {e}", throttle_duration_sec=2.0)
            return None

# ================= 任务 4：空间坐标抓取 (YOLOE-ONNX + 颜色加权融合) =================
    def process_task4_vision(self, img):
        try:
            img_flipped = cv2.rotate(img, cv2.ROTATE_180)
            h, w = img_flipped.shape[:2]

            # 1. 将大模型下发的目标统一转换为规范的标准 Key
            canonical_target = self.brain_intent_to_target.get(self.task4_target_object)
            if not canonical_target:
                self.get_logger().warn(f"⚠️ 无法识别的目标需求名称: {self.task4_target_object}")
                return

            # 2. 赛场现场颜色配置表
            expected_color_map = {
                "medicine_box": "红",
                "paper_cup": "紫",
                "bread": "绿"
            }
            expected_color = expected_color_map.get(canonical_target)

            W_YOLO = 1.0
            W_COLOR = 0.0

            if not hasattr(self, 'yolo_task4_model') or not hasattr(self, 'yolo_task4_planA_model'):
                self.get_logger().error("⚠️ 任务4 视觉模型未完全加载！")
                return

            # 🌟 累加扫描次数
            self.task4_scan_count += 1
            
            # 🌟 动态决定当前使用的模型和阈值
            if not self.task4_use_plan_a:
                current_model = self.yolo_task4_model
                current_conf = 0.1
                model_name = "Plan B (大模型)"
            else:
                current_model = self.yolo_task4_planA_model
                current_conf = 0.55
                model_name = "Plan A (特训模型)"

            self.get_logger().info(f"📸 [{model_name}] 第 {self.task4_scan_count} 次扫描，寻找目标: {canonical_target}...")

            # 3. 动态模型全图推理
            results = current_model(img_flipped, conf=current_conf, verbose=False)
            boxes = results[0].boxes

            best_score = 0.0
            best_yolo_conf = 0.0  
            target_box = None
            
            hsv_img = cv2.cvtColor(img_flipped, cv2.COLOR_BGR2HSV)

            # 4. 后台静默计算：遍历所有候选框进行打分
            for box in boxes:
                cls_id = int(box.cls[0].cpu().numpy())
                raw_label = results[0].names[cls_id].lower() 
                yolo_conf = float(box.conf[0].cpu().numpy())

                mapped_class = self.class_map.get(raw_label, "unknown")

                if mapped_class == canonical_target:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)
                    
                    color_score = 0.0
                    if expected_color:
                        obj_hsv = hsv_img[y1:y2, x1:x2]
                        color_ranges = {
                            "红": [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [179, 255, 255])],
                            "橙": [([11, 100, 100], [25, 255, 255])],
                            "黄": [([26, 100, 100], [34, 255, 255])],
                            "绿": [([35, 50, 40], [85, 255, 255])],
                            "青": [([86, 100, 100], [100, 255, 255])],
                            "蓝": [([101, 50, 50], [130, 255, 255])],
                            "紫": [([131, 50, 50], [155, 255, 255])]
                        }
                        
                        ranges = color_ranges.get(expected_color, [])
                        c_mask = np.zeros(obj_hsv.shape[:2], dtype=np.uint8)
                        for lower, upper in ranges:
                            partial_mask = cv2.inRange(obj_hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
                            c_mask = cv2.bitwise_or(c_mask, partial_mask)
                        
                        color_pixel_count = cv2.countNonZero(c_mask)
                        box_area = (x2 - x1) * (y2 - y1)
                        if box_area > 0:
                            color_ratio = color_pixel_count / float(box_area)
                            color_score = min(1.0, color_ratio / 0.15)
                    else:
                        color_score = yolo_conf

                    final_score = (yolo_conf * W_YOLO) + (color_score * W_COLOR)
                    
                    self.get_logger().info(f"🔍 候选评估 [{raw_label} -> {mapped_class}]: YOLO={yolo_conf:.2f}, 颜色={color_score:.2f} -> 综合={final_score:.2f}")

                    if final_score > best_score:
                        best_score = final_score
                        best_yolo_conf = yolo_conf
                        target_box = (x1, y1, x2, y2)

            # 🌟 核心及格线：包含动态阈值校验和模型切换逻辑
            if target_box is None or best_yolo_conf < current_conf:
                self.get_logger().warn(f"⚠️ {model_name} 未找到置信度 >= {current_conf} 的 [{canonical_target}]")
                
                # 检查是否需要切换到 Plan A
                if not self.task4_use_plan_a and self.task4_scan_count >= 10:
                    self.get_logger().warn("🔄 Plan B 已连续扫描 10 次失败，强制切换至 Plan A (特训模型)！")
                    self.task4_use_plan_a = True
                    self.task4_scan_count = 0  # 切换模型后重置计数器
                
                return # 放弃当前帧，等待下一帧

            self.get_logger().info(f"🏆 成功锁定目标: {canonical_target}, YOLO置信度: {best_yolo_conf:.2f}, 综合得分: {best_score:.2f}")

            # 📸 5. 目标确认！此时才开始在原图上画框并保存最终结果
            x1, y1, x2, y2 = target_box
            vis_img = img_flipped.copy()
            cv2.rectangle(vis_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 4)
            # 在图上标出类别和关键置信度分数
            info_text = f"TARGET: {canonical_target} Y:{best_yolo_conf:.2f} S:{best_score:.2f}"
            cv2.putText(vis_img, info_text, (int(x1), int(y1) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imwrite("/var/tmp/task4_grasp_target.jpg", vis_img)
            self.get_logger().info("📸 已生成并保存抓取目标的视觉确认图: /var/tmp/task4_grasp_target.jpg")

            # 6. 像素坐标转相机空间坐标与 TF2 变换
            u_flipped = int((x1 + x2) / 2.0)
            
            # 🌟 优化 1：物理重心下移！
            # 不要取 50% 的绝对中心，取 65% 偏下的位置（直击杯子肚子），彻底避开杯口和杯沿
            v_flipped = int(y1 + (y2 - y1) * 0.65)

            # 逆向转回倒置的原始画面坐标
            u = w - 1 - u_flipped
            v = h - 1 - v_flipped

            if self.latest_depth_image is None: 
                self.get_logger().warn("⚠️ 未获取到深度图！等待下一帧...")
                return
                
            max_y, max_x = self.latest_depth_image.shape
            
            # 🌟 新增防坑机制：如果 RGB 和 Depth 传感器的分辨率不同，必须进行等比例缩放映射！
            if max_x != w or max_y != h:
                u = int(u * (max_x / w))
                v = int(v * (max_y / h))

            # 🌟 优化 2：极小核心区采样 (Shrink ROI)
            # 把采样半径从 15% 缩小到 5% (即只取中心 10%x10% 的极小方块)，避免碰到背景边缘
            box_w = (x2 - x1) * (max_x / w)
            box_h = (y2 - y1) * (max_y / h)
            half_k_x = max(3, int(box_w * 0.05))
            half_k_y = max(3, int(box_h * 0.05))

            min_v, max_v = max(0, v - half_k_y), min(max_y, v + half_k_y + 1)
            min_u, max_u = max(0, u - half_k_x), min(max_x, u + half_k_x + 1)
            roi_depth = self.latest_depth_image[min_v:max_v, min_u:max_u]
            
            # 过滤掉为 0（无效反光）的深度，以及远于 2000mm 的异常深度
            valid_depths = roi_depth[(roi_depth > 0) & (roi_depth < 2000)]
            
            if len(valid_depths) == 0: 
                self.get_logger().warn(f"⚠️ 目标区域内深度全为无效！等待下一帧...", throttle_duration_sec=2.0)
                return
            
            # 🌟 优化 3：百分位数过滤 (剔除偏差大的离群点)
            # 去掉距离最远的 25% (可能是杯底或背景) 和最近的 25% (可能是噪点)，只保留中间最密集的 50%
            p25 = np.percentile(valid_depths, 25)
            p75 = np.percentile(valid_depths, 75)
            core_depths = valid_depths[(valid_depths >= p25) & (valid_depths <= p75)]
            
            if len(core_depths) == 0:  # 防御性回退机制
                core_depths = valid_depths
                
            # 从最稳定、没有极端偏差的核心数据中取中位数，完美抗噪
            z_depth_m = float(np.median(core_depths)) / 1000.0  
            
            if self.camera_intrinsics is None: 
                self.get_logger().warn("⚠️ 尚未获取到相机内参，无法解算！")
                return
                
            fx = self.camera_intrinsics.k[0] * (max_x / w) if max_x != w else self.camera_intrinsics.k[0]
            cx = self.camera_intrinsics.k[2] * (max_x / w) if max_x != w else self.camera_intrinsics.k[2]
            fy = self.camera_intrinsics.k[4] * (max_y / h) if max_y != h else self.camera_intrinsics.k[4]
            cy = self.camera_intrinsics.k[5] * (max_y / h) if max_y != h else self.camera_intrinsics.k[5]
            
            x_cam = (u - cx) * z_depth_m / fx
            y_cam = (v - cy) * z_depth_m / fy
            z_cam = z_depth_m
            
            try:
                transform = self.tf_buffer.lookup_transform('base_link', self.camera_intrinsics.header.frame_id, rclpy.time.Time())
                cam_pose = tf2_geometry_msgs.PoseStamped()
                cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z = x_cam, y_cam, z_cam
                cam_pose.pose.orientation.w = 1.0
                base_pose = tf2_geometry_msgs.do_transform_pose(cam_pose.pose, transform)
                base_pose.position.z = base_pose.position.z - 0.03
                base_pose.position.x = base_pose.position.x - 0.03
                grasp_msg = {
                    "object_name": canonical_target, 
                    "target_point": [base_pose.position.x, base_pose.position.y, base_pose.position.z]
                }
                msg = String()
                msg.data = json.dumps(grasp_msg, ensure_ascii=False)
                self.pub_grasp.publish(msg)
                
                self.get_logger().info(f"📤 [任务4] 成功向夹爪下发抓取坐标 -> X:{base_pose.position.x:.3f}, Y:{base_pose.position.y:.3f}, Z:{base_pose.position.z:.3f} (测距:{z_depth_m:.3f}m)")
                
                # 下发完成后重置任务锁，真正停止扫描
                self.current_task = None
                self.task4_target_object = ""

            except Exception as e:
                self.get_logger().error(f"❌ TF 转换失败: {e}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 任务 4 视觉处理失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()