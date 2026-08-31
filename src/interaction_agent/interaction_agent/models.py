#!/usr/bin/env python3

"""
interaction_agent 的公共数据模型。

所有发布到 /ai_agent/input_json 的 JSON 均严格包含：

{
    "intent_type": "...",
    "slots": {},
    "confidence": 1.0,
    "next_state": "..."
}

不再包含 source 字段。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Any


class State(str, Enum):
    """推理与交互状态。"""

    # 任务1等待参赛队员语音下达导航指令。
    TASK1_WAIT_COMMAND = "TASK1_WAIT_COMMAND"

    # 已识别任务1导航指令，
    # 等待导航执行模块完成交互区I就位。
    TASK1_NAVIGATING = "TASK1_NAVIGATING"

    # 后续常规交互监听。
    INTERACTION_LISTEN = "INTERACTION_LISTEN"

    # 已识别任务4唤醒，等待执行模块完成：
    # 关切表情 + “今天状态怎么样？”
    TASK4_ASK_STATUS = "TASK4_ASK_STATUS"

    # 允许识别任务4三类用户需求。
    TASK4_LISTEN_NEED = "TASK4_LISTEN_NEED"

    # 需求已经识别，执行模块正在完成：
    # 回复、导航、识别、抓取、返回和递交。
    TASK4_WAIT_SERVICE_COMPLETE = "TASK4_WAIT_SERVICE_COMPLETE"

    # 预留任务彻底结束状态。
    FINISH = "FINISH"


class IntentType(str, Enum):
    """对外发布的意图类型。"""

    TASK1_GO_INTERACTION_AREA = (
        "task1_go_interaction_area"
    )

    TASK3_TIME_QUERY = "task3_time_query"
    TASK3_DIGIT_COLOR_QUERY = "task3_digit_color_query"
    TASK3_EMOJI_CONTROL = "task3_emoji_control"
    TASK3_ACTION_CONTROL = "task3_action_control"

    TASK4_WAKE_SERVICE = "task4_wake_service"
    TASK4_NEED_CLASSIFICATION = "task4_need_classification"

    UNKNOWN = "unknown"


@dataclass(frozen=True)
class IntentResult:
    """一次闭集意图识别结果。"""

    intent_type: IntentType
    slots: dict[str, Any]
    confidence: float
    next_state: State

    def to_dict(self) -> dict[str, Any]:
        """转换为对外发布的四字段字典。"""
        return {
            "intent_type": self.intent_type.value,
            "slots": self.slots,
            "confidence": float(self.confidence),
            "next_state": self.next_state.value,
        }

    def to_json(self) -> str:
        """转换为不转义中文的 JSON 字符串。"""
        return json.dumps(
            self.to_dict(),
            ensure_ascii=False,
        )
