#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger


TARGET_TOPIC = "/vision/grasp_target_xyz"

EXECUTE_SERVICE = "/x2_arm/execute_latest_target"
CLEAR_SERVICE = "/x2_arm/clear_target"

# 第一阶段明确规定：
# 输入XYZ必须已经转换到当前IK模型坐标系。
EXPECTED_FRAME = "x2_ik_model"

# 防止很久以前的视觉结果突然被执行
TARGET_MAX_AGE_SEC = 30.0

# 只是做异常值过滤。
# 真正工作空间/距离限制仍由validated executor负责。
ABS_COORD_SANITY_LIMIT_M = 2.0

WORKSPACE = Path.home() / "star_agibot"

VALIDATED_EXECUTOR = (
    WORKSPACE
    / "scripts"
    / "x2_xyz_executor_validated.py"
)

LOG_DIR = (
    WORKSPACE
    / "logs"
    / "xyz_executor_server"
)


class XYZExecutorServer(Node):

    def __init__(self):
        super().__init__(
            "x2_xyz_executor_server"
        )

        LOG_DIR.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.latest_target = None
        self.latest_target_received_monotonic = None

        self.busy = False

        self.target_sub = (
            self.create_subscription(
                PointStamped,
                TARGET_TOPIC,
                self.on_target,
                10,
            )
        )

        self.execute_srv = (
            self.create_service(
                Trigger,
                EXECUTE_SERVICE,
                self.on_execute,
            )
        )

        self.clear_srv = (
            self.create_service(
                Trigger,
                CLEAR_SERVICE,
                self.on_clear,
            )
        )

        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            "X2 XYZ Executor Server READY"
        )
        self.get_logger().info(
            f"TARGET_TOPIC={TARGET_TOPIC}"
        )
        self.get_logger().info(
            f"EXPECTED_FRAME={EXPECTED_FRAME}"
        )
        self.get_logger().info(
            f"EXECUTE_SERVICE={EXECUTE_SERVICE}"
        )
        self.get_logger().info(
            "TARGET_RECEIVE_CAUSES_MOTION=false"
        )
        self.get_logger().info(
            "EXECUTION_REQUIRES_EXPLICIT_SERVICE=true"
        )
        self.get_logger().info(
            "========================================"
        )

    def on_target(
        self,
        msg: PointStamped,
    ):
        frame = (
            msg.header.frame_id.strip()
        )

        xyz = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )

        # ----------------------------------------
        # frame检查
        # ----------------------------------------

        if frame != EXPECTED_FRAME:

            self.get_logger().error(
                "TARGET_REJECTED=true"
            )

            self.get_logger().error(
                f"REASON=frame_id必须为{EXPECTED_FRAME}, "
                f"当前={frame!r}"
            )

            return

        # ----------------------------------------
        # 数字检查
        # ----------------------------------------

        if not all(
            math.isfinite(v)
            for v in xyz
        ):

            self.get_logger().error(
                "TARGET_REJECTED=true"
            )

            self.get_logger().error(
                "REASON=XYZ包含NaN或Inf"
            )

            return

        if any(
            abs(v)
            > ABS_COORD_SANITY_LIMIT_M
            for v in xyz
        ):

            self.get_logger().error(
                "TARGET_REJECTED=true"
            )

            self.get_logger().error(
                "REASON=XYZ超过2m异常值过滤范围"
            )

            return

        # ----------------------------------------
        # 只缓存，不运动
        # ----------------------------------------

        self.latest_target = xyz

        self.latest_target_received_monotonic = (
            time.monotonic()
        )

        self.get_logger().info(
            "TARGET_ACCEPTED=true"
        )

        self.get_logger().info(
            f"TARGET_FRAME={frame}"
        )

        self.get_logger().info(
            "TARGET_XYZ="
            f"[{xyz[0]:+.6f}, "
            f"{xyz[1]:+.6f}, "
            f"{xyz[2]:+.6f}]"
        )

        self.get_logger().info(
            "REAL_ROBOT_MOTION=false"
        )

        self.get_logger().info(
            "NEXT_STEP=call "
            f"{EXECUTE_SERVICE}"
        )

    def on_clear(
        self,
        request,
        response,
    ):
        if self.busy:

            response.success = False
            response.message = (
                "executor busy，不能清除目标"
            )

            return response

        self.latest_target = None
        self.latest_target_received_monotonic = None

        response.success = True
        response.message = (
            "latest target cleared"
        )

        self.get_logger().info(
            "TARGET_CLEARED=true"
        )

        return response

    def _run_executor(
        self,
        xyz,
        *,
        execute,
        log_file,
    ):
        command = [
            sys.executable,
            str(VALIDATED_EXECUTOR),
            "--target",
            f"{xyz[0]:.9f}",
            f"{xyz[1]:.9f}",
            f"{xyz[2]:.9f}",
        ]

        if execute:
            command.append(
                "--execute"
            )

        mode = (
            "REAL"
            if execute
            else "DRY_RUN"
        )

        self.get_logger().info(
            f"EXECUTOR_MODE={mode}"
        )

        self.get_logger().info(
            "EXECUTOR_TARGET_XYZ="
            f"[{xyz[0]:+.6f}, "
            f"{xyz[1]:+.6f}, "
            f"{xyz[2]:+.6f}]"
        )

        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        result_pass = False

        final_error_cm = None
        return_error_cm = None

        with log_file.open(
            "a",
            encoding="utf-8",
        ) as fp:

            fp.write(
                "\n========================================\n"
            )

            fp.write(
                f"MODE={mode}\n"
            )

            fp.write(
                "COMMAND="
                + " ".join(command)
                + "\n"
            )

            fp.flush()

            assert process.stdout is not None

            for raw_line in process.stdout:

                line = raw_line.rstrip()

                print(
                    f"[EXECUTOR] {line}",
                    flush=True,
                )

                fp.write(
                    line + "\n"
                )

                fp.flush()

                if line == "RESULT=PASS":
                    result_pass = True

                if line.startswith(
                    "FINAL_TARGET_ERROR_CM="
                ):
                    try:
                        final_error_cm = float(
                            line.split(
                                "=",
                                1,
                            )[1]
                        )
                    except ValueError:
                        pass

                if line.startswith(
                    "RETURN_XYZ_ERROR_CM="
                ):
                    try:
                        return_error_cm = float(
                            line.split(
                                "=",
                                1,
                            )[1]
                        )
                    except ValueError:
                        pass

        return_code = (
            process.wait()
        )

        return {
            "return_code": return_code,
            "result_pass": result_pass,
            "final_error_cm": final_error_cm,
            "return_error_cm": return_error_cm,
        }

    def on_execute(
        self,
        request,
        response,
    ):
        if self.busy:

            response.success = False
            response.message = (
                "executor currently busy"
            )

            return response

        if self.latest_target is None:

            response.success = False
            response.message = (
                "没有缓存视觉目标"
            )

            return response

        age = (
            time.monotonic()
            - self.latest_target_received_monotonic
        )

        if age > TARGET_MAX_AGE_SEC:

            response.success = False
            response.message = (
                "视觉目标已经过期，"
                f"age={age:.1f}s，"
                "请重新发送"
            )

            self.get_logger().warning(
                "TARGET_EXPIRED=true"
            )

            return response

        if not VALIDATED_EXECUTOR.exists():

            response.success = False
            response.message = (
                "validated executor不存在："
                f"{VALIDATED_EXECUTOR}"
            )

            return response

        xyz = tuple(
            self.latest_target
        )

        timestamp = (
            time.strftime(
                "%Y%m%d_%H%M%S"
            )
        )

        log_file = (
            LOG_DIR
            / f"execute_{timestamp}.log"
        )

        self.busy = True

        try:
            # ====================================
            # 第一阶段：
            # 自动DRY-RUN，不允许直接跳过。
            # ====================================

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "PREFLIGHT_START=true"
            )

            self.get_logger().info(
                "PREFLIGHT_REAL_ROBOT_MOTION=false"
            )

            dry = self._run_executor(
                xyz,
                execute=False,
                log_file=log_file,
            )

            if (
                dry["return_code"] != 0
                or not dry["result_pass"]
            ):
                self.get_logger().error(
                    "PREFLIGHT_RESULT=FAIL"
                )

                response.success = False
                response.message = (
                    "dry-run安全检查失败，"
                    "已拒绝真实运动"
                )

                return response

            self.get_logger().info(
                "PREFLIGHT_RESULT=PASS"
            )

            # ====================================
            # 第二阶段：真实执行
            # ====================================

            self.get_logger().warning(
                "REAL_EXECUTION_START=true"
            )

            self.get_logger().warning(
                "REAL_ROBOT_MOTION=true"
            )

            real = self._run_executor(
                xyz,
                execute=True,
                log_file=log_file,
            )

            if (
                real["return_code"] == 0
                and real["result_pass"]
            ):
                response.success = True

                parts = [
                    "RESULT=PASS"
                ]

                if (
                    real["final_error_cm"]
                    is not None
                ):
                    parts.append(
                        "target_error_cm="
                        f"{real['final_error_cm']:.3f}"
                    )

                if (
                    real["return_error_cm"]
                    is not None
                ):
                    parts.append(
                        "return_error_cm="
                        f"{real['return_error_cm']:.3f}"
                    )

                parts.append(
                    f"log={log_file}"
                )

                response.message = " | ".join(
                    parts
                )

                self.get_logger().info(
                    "EXECUTION_RESULT=PASS"
                )

            else:
                response.success = False

                response.message = (
                    "RESULT=FAIL | "
                    f"log={log_file}"
                )

                self.get_logger().error(
                    "EXECUTION_RESULT=FAIL"
                )

            return response

        except Exception as exc:

            response.success = False
            response.message = (
                f"executor server exception: {exc}"
            )

            self.get_logger().error(
                f"EXECUTION_EXCEPTION={exc}"
            )

            return response

        finally:
            self.busy = False


def main():

    rclpy.init()

    node = XYZExecutorServer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
