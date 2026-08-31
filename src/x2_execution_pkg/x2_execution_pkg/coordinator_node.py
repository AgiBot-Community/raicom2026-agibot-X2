#!/usr/bin/env python3
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from aimdk_msgs.srv import PlayEmoji, SetMcPresetMotion


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')
        self.get_logger().info("🧠 总指挥大脑启动中...")

        # ================================================================
        # 1. 官方动作 / 表情服务
        # ================================================================
        self.cli_emoji = self.create_client(
            PlayEmoji,
            '/aimdk_5Fmsgs/srv/PlayEmoji'
        )
        self.cli_motion = self.create_client(
            SetMcPresetMotion,
            '/aimdk_5Fmsgs/srv/SetMcPresetMotion'
        )

        # ================================================================
        # 2. 上游：interaction_agent
        # ================================================================
        self.sub_brain = self.create_subscription(
            String,
            '/ai_agent/input_json',
            self.brain_cb,
            10
        )

        # ================================================================
        # 3. 两套导航完全分开
        #    任务1：出发区 -> 交互区 I
        #    服务导航：交互区 -> 作业区
        # ================================================================
        self.sub_task1_nav = self.create_subscription(
            String,
            '/race_task/task1/nav_status',
            self.task1_nav_cb,
            10
        )
        self.sub_service_nav = self.create_subscription(
            String,
            '/race_task/service/nav_status',
            self.service_nav_cb,
            10
        )

        self.pub_task1_nav = self.create_publisher(
            PoseStamped,
            '/race_task/task1/goal_pose',
            10
        )
        self.pub_service_nav = self.create_publisher(
            PoseStamped,
            '/race_task/service/goal_pose',
            10
        )

        # ================================================================
        # 4. 内部语音 / 视觉通信 & 夹爪状态监听
        # ================================================================
        self.sub_vision_res = self.create_subscription(
            String,
            '/system/vision_result',
            self.vision_res_cb,
            10
        )
        
        # 🌟 核心新增：订阅夹爪的抓取完成状态话题（由夹爪同学发布）
        self.sub_grasp_status = self.create_subscription(
            String,
            '/competition/grasp_status',
            self.grasp_status_cb,
            10
        )

        self.pub_voice = self.create_publisher(
            String,
            '/system/voice_cmd',
            10
        )
        self.pub_vision = self.create_publisher(
            String,
            '/system/vision_cmd',
            10
        )

        # 控制 interaction_agent / ASR 是否继续听
        self.pub_listen = self.create_publisher(
            Bool,
            '/ai_agent/listen_control',
            10
        )

        # ================================================================
        # 5. 导航目标参数
        # ================================================================
        self.declare_parameter('task1_goal_configured', True)
        self.declare_parameter('task1_goal_x', 2.92)
        self.declare_parameter('task1_goal_y', -1.60)
        self.declare_parameter('task1_goal_yaw', 2.85)

        self.declare_parameter('service_goal_configured', True)
        self.declare_parameter('service_goal_x', -0.30)
        self.declare_parameter('service_goal_y', -2.83)
        self.declare_parameter('service_goal_yaw', -1.58)

        # ================================================================
        # 6. 运行状态
        # ================================================================
        self.current_service_target = ""
        self.pending_broadcast_text = ""  # 🌟 缓存等待夹爪完成后触发的标准播报文案
        self.task1_nav_active = False
        self.service_nav_active = False

        self.get_logger().info(
            "🧭 导航接口已分离："
            "task1=/race_task/task1/*，service=/race_task/service/*"
        )

    # ------------------------------------------------------------------
    # 通用输出
    # ------------------------------------------------------------------
    def speak(self, text, auto_open_mic=True):
        cmd_data = {
            "text": text,
            "auto_open_mic": auto_open_mic
        }
        self.pub_voice.publish(String(data=json.dumps(cmd_data, ensure_ascii=False)))

    def set_mic_status(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_listen.publish(msg)

    def play_emoji(self, emotion_id):
        if not self.cli_emoji.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("PlayEmoji 服务不可用")
            return

        req = PlayEmoji.Request()
        req.emotion_id = int(emotion_id)
        req.mode = 1
        req.priority = 100
        self.cli_emoji.call_async(req)

    def call_motion(self, motion_id, area_id):
        if not self.cli_motion.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SetMcPresetMotion 服务不可用")
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
        self.cli_motion.call_async(req)

    # ------------------------------------------------------------------
    # 导航辅助
    # ------------------------------------------------------------------
    def make_goal(self, x: float, y: float, yaw: float) -> PoseStamped:
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal_msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return goal_msg

    def publish_task1_goal(self):
        configured = bool(self.get_parameter('task1_goal_configured').value)
        if not configured:
            self.get_logger().error(
                "TASK1_GOAL_NOT_CONFIGURED："
                "尚未配置交互区 I 的 x/y/yaw，本次不会发布导航目标，机器人不会移动。"
            )
            return False

        x = float(self.get_parameter('task1_goal_x').value)
        y = float(self.get_parameter('task1_goal_y').value)
        yaw = float(self.get_parameter('task1_goal_yaw').value)

        self.pub_task1_nav.publish(self.make_goal(x, y, yaw))
        self.task1_nav_active = True

        self.get_logger().info(
            f"🧭 TASK1_GOAL_PUBLISHED: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )
        return True

    def publish_service_goal(self):
        configured = bool(self.get_parameter('service_goal_configured').value)
        if not configured:
            self.get_logger().error(
                "SERVICE_GOAL_NOT_CONFIGURED："
                "尚未配置作业区目标，本次不会发布服务导航目标。"
            )
            return False

        x = float(self.get_parameter('service_goal_x').value)
        y = float(self.get_parameter('service_goal_y').value)
        yaw = float(self.get_parameter('service_goal_yaw').value)

        self.pub_service_nav.publish(self.make_goal(x, y, yaw))
        self.service_nav_active = True

        self.get_logger().info(
            f"🧭 SERVICE_GOAL_PUBLISHED: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )
        return True

    # ------------------------------------------------------------------
    # interaction_agent 指令入口
    # ------------------------------------------------------------------
    def brain_cb(self, msg):
        try:
            brain_data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"input_json 解析失败: {exc}; raw={msg.data}")
            return

        intent = brain_data.get('intent_type', 'unknown')
        slots = brain_data.get('slots', {})
        self.get_logger().info(f"收到指令: {intent}")

        if intent == 'task1_go_interaction_area':
            if not bool(self.get_parameter('task1_goal_configured').value):
                self.get_logger().error(
                    "TASK1_GOAL_NOT_CONFIGURED：请先填写官方地图中的交互区 I 目标位姿。"
                )
                return

            self.set_mic_status(False)
            self.get_logger().info("🎤 TASK1 导航开始：已关闭语音输入")
            self.publish_task1_goal()

        elif intent == 'task3_time_query':
            self.speak(slots.get('reply_text', ''))

        elif intent == 'task3_emoji_control':
            self.play_emoji(slots.get('emotion_id', 90))
            self.set_mic_status(True)

        elif intent == 'task3_action_control':
            self.call_motion(
                slots.get('motion', 1002),
                slots.get('area', 1)
            )
            self.speak(slots.get('reply_text', ''))

        elif intent == 'task3_digit_color_query':
            self.pub_vision.publish(
                String(data=json.dumps({"task": "3"}, ensure_ascii=False))
            )

        elif intent == 'task4_wake_service':
            self.play_emoji(slots.get('emotion_id', 120))
            self.speak(slots.get('reply_text', '今天状态怎么样？'))

        elif intent == 'task4_need_classification':
            raw_target = slots.get('target_object', '')
            self.current_service_target = raw_target

            # 🌟 核心逻辑：根据大模型识别到的目标映射为赛题规则要求的标准播报文案
            if raw_target in ["药盒", "medicine_box"]:
                self.pending_broadcast_text = "已帮您拿到药盒"
            elif raw_target in ["纸杯", "paper_cup", "cup"]:
                self.pending_broadcast_text = "已帮您拿到水杯"
            elif raw_target in ["面包", "bread", "small bread"]:
                self.pending_broadcast_text = "已帮您拿到面包"
            else:
                self.pending_broadcast_text = f"已帮您拿到{raw_target}"
            
            # 回应完后关闭麦克风
            self.speak(
                slots.get('reply_text', '好的，这就去办。'),
                auto_open_mic=False
            )

            if slots.get('start_autonomous_service'):
                import threading # 局部导入，不影响其他代码
                
                # 定义延迟触发的导航函数
                def delay_nav():
                    self.get_logger().info("🧭 语音应答完成，正式前往作业区...")
                    self.publish_service_goal()
                
                self.get_logger().info("⏳ 正在播报语音，等待 4 秒后开始导航...")
                # 🌟 核心修改：设置 10.0 秒的非阻塞延迟（完美覆盖标准话术的播报时长），然后再触发导航
                threading.Timer(10.0, delay_nav).start()
            # 回应完“好的，这就去办”后关闭麦克风，前往作业区[cite: 3]

        elif intent == 'unknown':
            self.speak(slots.get('reply_text', '我没有听清楚。'))

        else:
            self.get_logger().warn(f"未处理的 intent_type: {intent}")

    # ------------------------------------------------------------------
    # 任务1导航结果：到达后进入比赛任务2
    # ------------------------------------------------------------------
    def task1_nav_cb(self, msg):
        status = msg.data.strip().lower()
        self.get_logger().info(f"TASK1_NAV_STATUS={status}")

        if status != 'reached' or not self.task1_nav_active:
            return

        self.task1_nav_active = False
        self.get_logger().info(
            "📍 TASK1_REACHED：已到达交互区 I；"
            "发布 listen_control=True，进入比赛任务2。"
        )
        self.set_mic_status(True)

    # ------------------------------------------------------------------
    # 服务导航结果：到达作业区后触发视觉找物品并交由夹爪
    # ------------------------------------------------------------------
    def service_nav_cb(self, msg):
        status = msg.data.strip().lower()
        self.get_logger().info(f"SERVICE_NAV_STATUS={status}")

        if (
            status == 'reached'
            and self.service_nav_active
            and self.current_service_target
        ):
            self.service_nav_active = False
            self.get_logger().info("📍 已到达作业区，开始让视觉找物品并发送给夹爪！")
            self.pub_vision.publish(
                String(data=json.dumps({
                    "task": "4",
                    "target": self.current_service_target
                }, ensure_ascii=False))
            )
            # 注意：此处清空 current_service_target 但保留 self.pending_broadcast_text 
            # 留给后续的 grasp_status_cb 触发最终播报
            self.current_service_target = ""

    # ------------------------------------------------------------------
    # 🌟 核心新增：监听夹爪抓取完成状态的回调函数
    # ------------------------------------------------------------------
    def grasp_status_cb(self, msg):
        status_data = msg.data.strip().lower()
        # 接收夹爪发出的成功信号（支持 "success" 或 "done"）
        if status_data in ["success", "done"] and self.pending_broadcast_text:
            self.get_logger().info(f"🦾 收到夹爪抓取成功信号！触发最终播报: {self.pending_broadcast_text}")
            
            # 根据赛题要求：播报完成即视为本任务结束，因此设置 auto_open_mic=False[cite: 3, 6]
            self.speak(self.pending_broadcast_text, auto_open_mic=False)
            
            # 清空缓存文案，避免重复触发
            self.pending_broadcast_text = ""

    def vision_res_cb(self, msg):
        try:
            res = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"vision_result 解析失败: {exc}; raw={msg.data}")
            return

        if res.get('task') == '3':
            self.speak(res.get('result', ''))


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()