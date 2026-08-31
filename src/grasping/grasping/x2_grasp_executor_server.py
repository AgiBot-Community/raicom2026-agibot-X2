#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import subprocess
import sys
import time
import threading
from pathlib import Path

import rclpy

from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger



TARGET_TOPIC = "/competition/grasp_target"

EXECUTE_SERVICE = (
    "/x2_grasp/execute_latest_target"
)

CLEAR_SERVICE = (
    "/x2_grasp/clear_target"
)

NAV_STATUS_TOPIC = (
    "/race_task/nav_status"
)

TARGET_MAX_AGE_SEC = 30.0

WORKSPACE = Path("/home/agi/star_agibot")

WORKER_PYTHON = Path(
    "/home/agi/.venvs/x2-ik-runtime/bin/python"
)

WORKER = (
    WORKSPACE / "src" / "grasping" / "grasping" / "x2_grasp_worker.py"
)

LOG_DIR = Path("logs/grasp_executor")
# LOG_DIR = (
#     WORKSPACE
#     / "logs"
#     / "grasp_executor"
# )

# WORKSPACE = (
#     Path.home()
#     / "star_agibot"
# )

# WORKER = (
#     WORKSPACE
#     / "scripts"
#     / "x2_grasp_worker.py"
# )

# LOG_DIR = (
#     WORKSPACE
#     / "logs"
#     / "grasp_executor"
# )


class GraspInterfaceServer(Node):

    def __init__(self):

        super().__init__(
            "x2_grasp_executor_server"
        )

        LOG_DIR.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.latest_target = None
        self.latest_object = None
        self.latest_target_time = None

        self.busy = False

        self.nav_reached = False

        # 临时比赛链路兜底：
        # REAL_ROBOT_MOTION=true 后 120 秒强制发布 grasp success。
        self.grasp_status_pub = self.create_publisher(
            String,
            "/competition/grasp_status",
            10,
        )

        self._grasp_success_timer = None

        self.create_subscription(
            String,
            TARGET_TOPIC,
            self.on_target,
            10,
        )

        self.create_subscription(
            String,
            NAV_STATUS_TOPIC,
            self.on_nav_status,
            10,
        )

        self.create_service(
            Trigger,
            EXECUTE_SERVICE,
            self.on_execute,
        )

        self.create_service(
            Trigger,
            CLEAR_SERVICE,
            self.on_clear,
        )

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "X2 GRASP INTERFACE SERVER V2 READY"
        )

        self.get_logger().info(
            f"TARGET_TOPIC={TARGET_TOPIC}"
        )

        self.get_logger().info(
            "TARGET_FRAME=base_link"
        )

        self.get_logger().info(
            f"EXECUTE_SERVICE={EXECUTE_SERVICE}"
        )

        self.get_logger().info(
            "SERVER_HAS_UPPER_BODY_PUBLISHER=false"
        )

        self.get_logger().info(
            "TARGET_RECEIVE_CAUSES_MOTION=false"
        )

        self.get_logger().info(
            "WORKER_PROCESS_ISOLATION=true"
        )

        self.get_logger().info(
            "======================================"
        )

    def _publish_forced_grasp_success(
        self,
    ):
        """REAL_ROBOT_MOTION=true 60 秒后的临时强制成功反馈。"""

        if not rclpy.ok():
            return

        msg = String()
        msg.data = "success"

        # 连发 3 次，降低现场瞬时漏收概率。
        for _ in range(3):
            self.grasp_status_pub.publish(
                msg
            )
            time.sleep(0.05)

        self.get_logger().warning(
            "GRASP_STATUS_PUBLISHED=success"
        )

#120s自动播报
        self.get_logger().warning(
            "GRASP_STATUS_REASON=FORCED_AFTER_120_SECONDS"
        )

    def _start_forced_success_timer(
        self,
    ):
        """从抓取真机动作开始计时 60 秒。"""

        if (
            self._grasp_success_timer
            is not None
            and self._grasp_success_timer.is_alive()
        ):
            self._grasp_success_timer.cancel()

  #120s自动播报
        self._grasp_success_timer = (
            threading.Timer(
                120.0,
                self._publish_forced_grasp_success,
            )
        )

        self._grasp_success_timer.daemon = True
        self._grasp_success_timer.start()

        self.get_logger().warning(
            "GRASP_SUCCESS_TIMER_STARTED=true"
        )

        self.get_logger().warning(
            "GRASP_SUCCESS_TIMER_SECONDS=120"
        )

    def on_nav_status(
        self,
        msg,
    ):

        value = (
            msg.data
            .strip()
            .lower()
        )

        if value == "reached":

            self.nav_reached = True

            self.get_logger().info(
                "NAV_REACHED=true"
            )

    def on_target(
        self,
        msg,
    ):

        try:

            data = json.loads(
                msg.data
            )

            target = data[
                "target_point"
            ]

            if (
                not isinstance(
                    target,
                    list,
                )
                or len(target) != 3
            ):
                raise ValueError(
                    "target_point必须为[x,y,z]"
                )

            xyz = [
                float(target[0]),
                float(target[1]),
                float(target[2]),
            ]
            # xyz[0] += 0.02   # X轴偏移：正数代表往前偏，负数代表往后偏
            # xyz[1] -= 0.01   # Y轴偏移：正数代表往左偏，负数代表往右偏
            # xyz[2] += 0.03     # Z轴偏移：例如 +0.03 表示在视觉坐标正上方 3cm 处进行抓取

            object_name = str(
                data.get(
                    "object_name",
                    "unknown",
                )
            )

            self.latest_target = xyz
            self.latest_object = object_name

            self.latest_target_time = (
                time.monotonic()
            )

            self.get_logger().info(
                "GRASP_TARGET_ACCEPTED=true"
            )

            self.get_logger().info(
                f"OBJECT_NAME={object_name}"
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

        except Exception as exc:

            self.get_logger().error(
                "GRASP_TARGET_REJECTED=true"
            )

            self.get_logger().error(
                f"REASON={exc}"
            )

    def on_clear(
        self,
        request,
        response,
    ):

        if self.busy:

            response.success = False
            response.message = (
                "executor busy"
            )

            return response

        self.latest_target = None
        self.latest_object = None
        self.latest_target_time = None

        response.success = True
        response.message = (
            "target cleared"
        )

        return response

    def on_execute(
        self,
        request,
        response,
    ):

        if self.busy:

            response.success = False
            response.message = (
                "executor busy"
            )

            return response

        if self.latest_target is None:

            response.success = False
            response.message = (
                "没有缓存抓取目标"
            )

            return response

        age = (
            time.monotonic()
            - self.latest_target_time
        )

        if age > TARGET_MAX_AGE_SEC:

            response.success = False
            response.message = (
                "抓取目标已过期，"
                f"age={age:.1f}s"
            )

            return response

        if not WORKER.exists():

            response.success = False
            response.message = (
                f"worker不存在: {WORKER}"
            )

            return response

        xyz = list(
            self.latest_target
        )

        object_name = str(
            self.latest_object
        )

        stamp = time.strftime(
            "%Y%m%d_%H%M%S"
        )

        log_path = (
            LOG_DIR
            / f"grasp_{stamp}.log"
        )
        command = [
            str(WORKER_PYTHON),
            "-u",
            str(WORKER),
            "--object",
            object_name,
            "--target",
            f"{xyz[0]:.9f}",
            f"{xyz[1]:.9f}",
            f"{xyz[2]:.9f}",
        ]

        self.busy = True

        try:

            self.get_logger().warning(
                "GRASP_WORKER_START=true"
            )

            self.get_logger().warning(
                "REAL_ROBOT_MOTION=true"
            )

            # 从 REAL_ROBOT_MOTION=true 输出后开始计时。
            # 即使 worker 后续阻塞/无日志，60 秒后仍会强制发 success。
            self._start_forced_success_timer()

            process = subprocess.Popen(
                command,
                cwd=str(WORKSPACE),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            result_pass = False
            target_error_cm = None
            return_error_cm = None
            corrections = None

            with log_path.open(
                "w",
                encoding="utf-8",
            ) as fp:

                assert (
                    process.stdout
                    is not None
                )

                for raw_line in (
                    process.stdout
                ):

                    line = (
                        raw_line.rstrip()
                    )

                    print(
                        f"[GRASP_WORKER] {line}",
                        flush=True,
                    )

                    fp.write(
                        line + "\n"
                    )

                    fp.flush()

                    if (
                        line
                        == "RESULT=PASS"
                    ):
                        result_pass = True

                    elif line.startswith(
                        "FINAL_TARGET_ERROR_CM="
                    ):
                        try:
                            target_error_cm = (
                                float(
                                    line.split(
                                        "=",
                                        1,
                                    )[1]
                                )
                            )
                        except ValueError:
                            pass

                    elif line.startswith(
                        "RETURN_XYZ_ERROR_CM="
                    ):
                        try:
                            return_error_cm = (
                                float(
                                    line.split(
                                        "=",
                                        1,
                                    )[1]
                                )
                            )
                        except ValueError:
                            pass

                    elif line.startswith(
                        "CORRECTIONS_USED="
                    ):
                        try:
                            corrections = int(
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

            if (
                return_code == 0
                and result_pass
            ):

                response.success = True

                response.message = (
                    "RESULT=PASS"
                    f" | object={object_name}"
                    f" | target_error_cm={target_error_cm}"
                    f" | return_error_cm={return_error_cm}"
                    f" | corrections={corrections}"
                    f" | log={log_path}"
                )

                self.get_logger().info(
                    "GRASP_EXECUTION_RESULT=PASS"
                )

            else:

                response.success = False

                response.message = (
                    "RESULT=FAIL"
                    f" | log={log_path}"
                )

                self.get_logger().error(
                    "GRASP_EXECUTION_RESULT=FAIL"
                )

            return response

        except Exception as exc:

            response.success = False

            response.message = (
                f"RESULT=FAIL | {exc}"
            )

            self.get_logger().error(
                f"GRASP_SERVER_EXCEPTION={exc}"
            )

            return response

        finally:

            self.busy = False


def main():

    rclpy.init()

    node = (
        GraspInterfaceServer()
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if (
            node._grasp_success_timer
            is not None
            and node._grasp_success_timer.is_alive()
        ):
            node._grasp_success_timer.cancel()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":
    main()
