#!/usr/bin/env python3

"""
根据当前交互状态执行闭集意图分流。
"""

from __future__ import annotations

from .models import IntentResult, State
from .rules import (
    build_action_result,
    build_task1_go_interaction_area_result,
    build_task1_unknown_result,
    build_digit_color_result,
    build_emoji_result,
    build_task3_unknown_result,
    build_task4_need_result,
    build_task4_unknown_result,
    build_task4_wake_result,
    build_time_result,
    is_digit_color_query,
    is_task1_go_interaction_area,
    is_task4_wake,
    is_time_query,
    match_action,
    match_emoji,
    match_task4_need,
    normalize_text,
)


class IntentDispatcher:
    """闭集意图分流器。"""

    def dispatch(
        self,
        text: str,
        current_state: State,
    ) -> IntentResult:
        """
        根据当前状态识别一条 ASR 文本。

        任务4唤醒仅在 INTERACTION_LISTEN 中识别。
        任务4需求仅在 ASK_STATUS / LISTEN_NEED 中识别。
        """
        normalized = normalize_text(text)

        if current_state == State.TASK1_WAIT_COMMAND:
            return self._dispatch_task1_wait_command(
                normalized
            )

        if current_state == State.INTERACTION_LISTEN:
            return self._dispatch_interaction_listen(
                normalized
            )

        if current_state in (
            State.TASK4_ASK_STATUS,
            State.TASK4_LISTEN_NEED,
        ):
            # 正常情况下，执行模块发布 True 后，
            # 状态应先从 ASK_STATUS 切换到 LISTEN_NEED。
            #
            # 同时允许 ASK_STATUS 接收需求文本，用于保护
            # 不同 ROS2 订阅回调到达顺序造成的极端竞态。
            return self._dispatch_task4_need(normalized)

        raise RuntimeError(
            "当前状态不接受语音输入："
            f"{current_state.value}"
        )

    @staticmethod
    def accepts_text(current_state: State) -> bool:
        """当前状态是否允许处理识别文本。"""
        return current_state in (
            State.TASK1_WAIT_COMMAND,
            State.INTERACTION_LISTEN,
            State.TASK4_ASK_STATUS,
            State.TASK4_LISTEN_NEED,
        )

    def _dispatch_task1_wait_command(
        self,
        text: str,
    ) -> IntentResult:
        """识别任务1前往交互区I指令。"""

        if not text:
            return build_task1_unknown_result()

        if is_task1_go_interaction_area(text):
            return (
                build_task1_go_interaction_area_result()
            )

        return build_task1_unknown_result()

    def _dispatch_interaction_listen(
        self,
        text: str,
    ) -> IntentResult:
        """
        任务3常规意图和任务4唤醒。

        任务4唤醒优先级最高。
        """
        if not text:
            return build_task3_unknown_result()

        # 任务4唤醒最高优先级。
        if is_task4_wake(text):
            return build_task4_wake_result()

        if is_time_query(text):
            return build_time_result()

        if is_digit_color_query(text):
            return build_digit_color_result()

        emoji_slots = match_emoji(text)

        if emoji_slots is not None:
            return build_emoji_result(emoji_slots)

        action_slots = match_action(text)

        if action_slots is not None:
            return build_action_result(action_slots)

        return build_task3_unknown_result()

    def _dispatch_task4_need(
        self,
        text: str,
    ) -> IntentResult:
        """识别任务4三类服务需求。"""
        if not text:
            return build_task4_unknown_result()

        need_slots = match_task4_need(text)

        if need_slots is not None:
            return build_task4_need_result(need_slots)

        return build_task4_unknown_result()
