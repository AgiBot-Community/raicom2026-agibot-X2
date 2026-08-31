#!/usr/bin/env python3

"""
interaction_agent ROS2节点。

订阅：
    /ai_agent/recognized_text
    std_msgs/msg/String

    /ai_agent/listen_control
    std_msgs/msg/Bool

发布：
    /ai_agent/input_json
    std_msgs/msg/String
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .models import State
from .state_machine import InteractionStateMachine


DEFAULT_RECOGNIZED_TEXT_TOPIC = (
    "/ai_agent/recognized_text"
)

DEFAULT_INPUT_JSON_TOPIC = (
    "/ai_agent/input_json"
)

DEFAULT_LISTEN_CONTROL_TOPIC = (
    "/ai_agent/listen_control"
)


class InteractionNode(Node):
    """语音文本到标准意图JSON的ROS2节点。"""

    def __init__(self) -> None:
        super().__init__("interaction_node")

        self._declare_parameters()
        self._read_parameters()

        self._state_machine = InteractionStateMachine()

        self._input_json_publisher = (
            self.create_publisher(
                String,
                self._input_json_topic,
                10,
            )
        )

        self._recognized_text_subscription = (
            self.create_subscription(
                String,
                self._recognized_text_topic,
                self._on_recognized_text,
                10,
            )
        )

        # 与voice_asr共同订阅该话题。
        #
        # voice_asr：
        #   True -> 开启麦克风
        #
        # interaction_agent：
        #   True -> 同步执行完成事件。
        #
        #   TASK1_NAVIGATING
        #       -> INTERACTION_LISTEN
        #
        #   TASK4_ASK_STATUS
        #       -> TASK4_LISTEN_NEED
        self._listen_control_subscription = (
            self.create_subscription(
                Bool,
                self._listen_control_topic,
                self._on_listen_control,
                10,
            )
        )

        self.get_logger().info(
            "interaction_node 初始化完成。"
        )

        self.get_logger().info(
            f"识别文本输入："
            f"{self._recognized_text_topic}"
        )

        self.get_logger().info(
            f"监听控制同步："
            f"{self._listen_control_topic}"
        )

        self.get_logger().info(
            f"意图JSON输出："
            f"{self._input_json_topic}"
        )

        self.get_logger().info(
            "初始状态：TASK1_WAIT_COMMAND"
        )

    def _declare_parameters(self) -> None:
        """声明ROS2参数。"""
        self.declare_parameter(
            "recognized_text_topic",
            DEFAULT_RECOGNIZED_TEXT_TOPIC,
        )

        self.declare_parameter(
            "input_json_topic",
            DEFAULT_INPUT_JSON_TOPIC,
        )

        self.declare_parameter(
            "listen_control_topic",
            DEFAULT_LISTEN_CONTROL_TOPIC,
        )

    def _read_parameters(self) -> None:
        """读取ROS2参数。"""
        self._recognized_text_topic = str(
            self.get_parameter(
                "recognized_text_topic"
            ).value
        )

        self._input_json_topic = str(
            self.get_parameter(
                "input_json_topic"
            ).value
        )

        self._listen_control_topic = str(
            self.get_parameter(
                "listen_control_topic"
            ).value
        )

        for name, value in (
            (
                "recognized_text_topic",
                self._recognized_text_topic,
            ),
            (
                "input_json_topic",
                self._input_json_topic,
            ),
            (
                "listen_control_topic",
                self._listen_control_topic,
            ),
        ):
            if not value:
                raise ValueError(
                    f"{name}不能为空"
                )

    def _on_recognized_text(
        self,
        message: String,
    ) -> None:
        """处理ASR发布的最终文字。"""
        text = message.data.strip()

        if not text:
            self.get_logger().warning(
                "收到空的recognized_text，已忽略。"
            )
            return

        state_before = (
            self._state_machine.current_state
        )

        self.get_logger().info(
            f"收到识别文本：{text}"
        )

        self.get_logger().info(
            f"处理前状态：{state_before.value}"
        )

        result = self._state_machine.handle_text(
            text
        )

        if result is None:
            self.get_logger().warning(
                "当前状态不接受语音输入，"
                f"已忽略文本：{text}；"
                "当前状态："
                f"{state_before.value}"
            )
            return

        payload = result.to_json()

        output_message = String()
        output_message.data = payload

        self._input_json_publisher.publish(
            output_message
        )

        self.get_logger().info(
            f"已发布 {self._input_json_topic}："
            f"{payload}"
        )

        self.get_logger().info(
            "状态变化："
            f"{state_before.value}"
            " -> "
            f"{result.next_state.value}"
        )

    def _on_listen_control(
        self,
        message: Bool,
    ) -> None:
        """
        同步执行模块发出的监听控制事件。

        True不由本节点产生，而由执行调度模块产生。
        """
        old_state, new_state = (
            self._state_machine.on_listen_control(
                message.data
            )
        )

        if message.data:
            if (
                old_state
                == State.TASK1_NAVIGATING
                and new_state
                == State.INTERACTION_LISTEN
            ):
                self.get_logger().info(
                    "任务1导航完成，"
                    "状态变化："
                    "TASK1_NAVIGATING"
                    " -> "
                    "INTERACTION_LISTEN"
                )
                return

            if (
                old_state
                == State.TASK4_ASK_STATUS
                and new_state
                == State.TASK4_LISTEN_NEED
            ):
                self.get_logger().info(
                    "执行模块已完成任务4询问，"
                    "状态变化："
                    "TASK4_ASK_STATUS"
                    " -> "
                    "TASK4_LISTEN_NEED"
                )
                return

            if old_state in (
                State.TASK4_WAIT_SERVICE_COMPLETE,
                State.FINISH,
            ):
                self.get_logger().warning(
                    "当前处于任务4自主服务或结束状态，"
                    "原则上不应发布listen_control=true。"
                )
                return

            self.get_logger().info(
                "收到listen_control=true，"
                "当前业务状态保持："
                f"{new_state.value}"
            )
            return

        self.get_logger().info(
            "收到listen_control=false，"
            "业务状态保持："
            f"{new_state.value}"
        )


def main(args=None) -> None:
    """ROS2节点入口。"""
    rclpy.init(args=args)

    node: InteractionNode | None = None

    try:
        node = InteractionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
