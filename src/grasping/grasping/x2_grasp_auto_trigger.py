#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
比赛任务4：视觉抓取目标自动执行触发器

输入：
    /competition/grasp_target
    std_msgs/msg/String

格式：
    {
        "object_name": "1",
        "target_point": [x, y, z]
    }

target_point:
    base_link 下的绝对 XYZ，单位 m

执行：
    收到合法目标后，自动调用：
    /x2_grasp/execute_latest_target

说明：
    真正机械臂动作仍由：
    x2_grasp_executor_server.py
        ->
    x2_grasp_worker.py
    执行。
"""

import json
import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger


TARGET_TOPIC = "/competition/grasp_target"

EXECUTE_SERVICE = (
    "/x2_grasp/execute_latest_target"
)

# 给 grasp_executor_server 一点时间，
# 先缓存同一条 /competition/grasp_target。
AUTO_TRIGGER_DELAY_SEC = 1.00


class GraspAutoTrigger(Node):

    def __init__(self):

        super().__init__(
            "x2_grasp_auto_trigger"
        )

        # -----------------------------------------
        # 状态
        # -----------------------------------------

        self.busy = False
        self.pending_timer = None

        self.pending_object_name = None
        self.pending_target_xyz = None

        # -----------------------------------------
        # 接收视觉目标
        # -----------------------------------------

        self.target_sub = (
            self.create_subscription(
                String,
                TARGET_TOPIC,
                self.on_target,
                10,
            )
        )

        # -----------------------------------------
        # 调用现有抓取服务
        # -----------------------------------------

        self.execute_client = (
            self.create_client(
                Trigger,
                EXECUTE_SERVICE,
            )
        )

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "X2 GRASP AUTO TRIGGER READY"
        )

        self.get_logger().info(
            f"TARGET_TOPIC={TARGET_TOPIC}"
        )

        self.get_logger().info(
            f"EXECUTE_SERVICE={EXECUTE_SERVICE}"
        )

        self.get_logger().info(
            "TARGET_FRAME=base_link"
        )

        self.get_logger().info(
            "AUTO_EXECUTE=true"
        )

        self.get_logger().info(
            "TARGET_RECEIVE_CAUSES_REAL_MOTION=true"
        )

        self.get_logger().info(
            "======================================"
        )

    # =============================================
    # 视觉目标
    # =============================================

    def on_target(self, msg):

        # 正在执行上一次抓取时，
        # 不允许并发启动第二个 worker。
        if self.busy:

            self.get_logger().warning(
                "GRASP_TARGET_IGNORED_BUSY=true"
            )

            return

        # -----------------------------------------
        # 解析 JSON
        # -----------------------------------------

        try:

            data = json.loads(
                msg.data
            )

        except Exception as exc:

            self.get_logger().error(
                "GRASP_TARGET_JSON_INVALID=true"
            )

            self.get_logger().error(
                f"REASON={exc}"
            )

            return

        if not isinstance(
            data,
            dict,
        ):

            self.get_logger().error(
                "GRASP_TARGET_INVALID=true"
            )

            self.get_logger().error(
                "REASON=JSON不是object"
            )

            return

        if (
            "object_name" not in data
            or "target_point" not in data
        ):

            self.get_logger().error(
                "GRASP_TARGET_INVALID=true"
            )

            self.get_logger().error(
                "REASON=缺少object_name或target_point"
            )

            return

        target = data[
            "target_point"
        ]

        if (
            not isinstance(target, list)
            or len(target) != 3
        ):

            self.get_logger().error(
                "GRASP_TARGET_INVALID=true"
            )

            self.get_logger().error(
                "REASON=target_point必须为[x,y,z]"
            )

            return

        try:

            xyz = [
                float(target[0]),
                float(target[1]),
                float(target[2]),
            ]

        except Exception:

            self.get_logger().error(
                "GRASP_TARGET_INVALID=true"
            )

            self.get_logger().error(
                "REASON=XYZ无法转换为float"
            )

            return

        if not all(
            math.isfinite(v)
            for v in xyz
        ):

            self.get_logger().error(
                "GRASP_TARGET_INVALID=true"
            )

            self.get_logger().error(
                "REASON=XYZ包含NaN或Inf"
            )

            return

        object_name = str(
            data["object_name"]
        )

        # -----------------------------------------
        # 合法目标
        # -----------------------------------------

        self.pending_object_name = (
            object_name
        )

        self.pending_target_xyz = xyz

        self.busy = True

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "VISION_GRASP_TARGET_RECEIVED=true"
        )

        self.get_logger().info(
            f"OBJECT_NAME={object_name}"
        )

        self.get_logger().info(
            "TARGET_FRAME=base_link"
        )

        self.get_logger().info(
            "TARGET_XYZ="
            f"[{xyz[0]:+.6f}, "
            f"{xyz[1]:+.6f}, "
            f"{xyz[2]:+.6f}]"
        )

        self.get_logger().info(
            "AUTO_GRASP_SCHEDULED=true"
        )

        self.get_logger().info(
            "REAL_ROBOT_MOTION_IMMINENT=true"
        )

        self.get_logger().info(
            "======================================"
        )

        # -----------------------------------------
        # 延迟一点点调用服务
        #
        # 因为 grasp_executor_server 和本节点
        # 同时订阅 /competition/grasp_target。
        #
        # 先让 server 完成 target cache，
        # 再触发 execute service。
        # -----------------------------------------

        self.pending_timer = (
            self.create_timer(
                AUTO_TRIGGER_DELAY_SEC,
                self.trigger_grasp,
            )
        )

    # =============================================
    # 自动触发抓取
    # =============================================

    def trigger_grasp(self):

        # one-shot timer
        if self.pending_timer is not None:

            self.pending_timer.cancel()

            self.destroy_timer(
                self.pending_timer
            )

            self.pending_timer = None

        # -----------------------------------------
        # 服务检查
        # -----------------------------------------

        if not (
            self.execute_client
            .service_is_ready()
        ):

            self.get_logger().error(
                "AUTO_GRASP_START=false"
            )

            self.get_logger().error(
                "REASON="
                "抓取执行服务不可用："
                f"{EXECUTE_SERVICE}"
            )

            self.busy = False

            return

        # -----------------------------------------
        # 调用抓取服务
        # -----------------------------------------

        self.get_logger().info(
            "AUTO_GRASP_START=true"
        )

        self.get_logger().info(
            "REAL_ROBOT_MOTION=true"
        )

        request = Trigger.Request()

        future = (
            self.execute_client
            .call_async(request)
        )

        future.add_done_callback(
            self.on_grasp_done
        )

    # =============================================
    # 抓取结束
    # =============================================

    def on_grasp_done(
        self,
        future,
    ):

        try:

            response = (
                future.result()
            )

        except Exception as exc:

            self.get_logger().error(
                "AUTO_GRASP_RESULT=FAIL"
            )

            self.get_logger().error(
                f"REASON={exc}"
            )

            self.busy = False

            return

        if response is None:

            self.get_logger().error(
                "AUTO_GRASP_RESULT=FAIL"
            )

            self.get_logger().error(
                "REASON=service response为空"
            )

            self.busy = False

            return

        if response.success:

            self.get_logger().info(
                "======================================"
            )

            self.get_logger().info(
                "AUTO_GRASP_RESULT=PASS"
            )

            self.get_logger().info(
                f"DETAIL={response.message}"
            )

            self.get_logger().info(
                "======================================"
            )

        else:

            self.get_logger().error(
                "======================================"
            )

            self.get_logger().error(
                "AUTO_GRASP_RESULT=FAIL"
            )

            self.get_logger().error(
                f"DETAIL={response.message}"
            )

            self.get_logger().error(
                "======================================"
            )

        self.pending_object_name = None
        self.pending_target_xyz = None

        self.busy = False


def main():

    rclpy.init()

    node = GraspAutoTrigger()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print(
            "\nAUTO_TRIGGER_STOPPED=true"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()
