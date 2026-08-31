#!/usr/bin/env python3

"""
interaction_agent 状态机。

状态变化来源：

1. 收到 /ai_agent/recognized_text
   → IntentDispatcher识别
   → 使用结果中的next_state更新状态

2. 收到 /ai_agent/listen_control = True
   → 表示执行模块已经完成上一轮动作或TTS
   → 若当前为TASK4_ASK_STATUS，则进入TASK4_LISTEN_NEED
"""

from __future__ import annotations

from threading import RLock

from .intent_dispatcher import IntentDispatcher
from .models import IntentResult, State


class InteractionStateMachine:
    """推理与交互状态机。"""

    def __init__(
        self,
        dispatcher: IntentDispatcher | None = None,
    ) -> None:
        self._dispatcher = (
            dispatcher
            if dispatcher is not None
            else IntentDispatcher()
        )

        self._current_state = (
            State.TASK1_WAIT_COMMAND
        )
        self._lock = RLock()

    @property
    def current_state(self) -> State:
        """获取当前状态。"""
        with self._lock:
            return self._current_state

    def handle_text(
        self,
        text: str,
    ) -> IntentResult | None:
        """
        处理一条 ASR 最终文本。

        WAIT_SERVICE_COMPLETE 和 FINISH 状态不会继续处理语音。
        """
        with self._lock:
            state_before = self._current_state

            if not self._dispatcher.accepts_text(
                state_before
            ):
                return None

            result = self._dispatcher.dispatch(
                text=text,
                current_state=state_before,
            )

            self._current_state = result.next_state

            return result

    def on_listen_control(
        self,
        enabled: bool,
    ) -> tuple[State, State]:
        """
        同步执行模块发出的监听控制事件。

        True：
            执行模块已完成上一轮执行并允许继续听用户说话。

            若当前状态为TASK4_ASK_STATUS，
            则进入TASK4_LISTEN_NEED。

        False：
            仅表示取消或关闭麦克风，
            不修改业务状态。
        """
        with self._lock:
            old_state = self._current_state

            if (
                enabled
                and self._current_state
                == State.TASK1_NAVIGATING
            ):
                # 任务1导航执行完成。
                # 同一条listen_control=true同时会被
                # voice_asr接收，从而开启下一轮监听。
                self._current_state = (
                    State.INTERACTION_LISTEN
                )

            elif (
                enabled
                and self._current_state
                == State.TASK4_ASK_STATUS
            ):
                self._current_state = (
                    State.TASK4_LISTEN_NEED
                )

            return old_state, self._current_state

    def mark_service_complete(
        self,
        *,
        return_to_interaction: bool = False,
    ) -> State:
        """
        预留给未来的自主服务完成反馈。

        当前队友尚未定义服务完成话题，因此本方法暂不连接ROS2。
        """
        with self._lock:
            if return_to_interaction:
                self._current_state = (
                    State.INTERACTION_LISTEN
                )
            else:
                self._current_state = State.FINISH

            return self._current_state

    def reset(self) -> State:
        """重置到比赛开始前的任务1等待状态。"""
        with self._lock:
            self._current_state = (
                State.TASK1_WAIT_COMMAND
            )

            return self._current_state
