#!/usr/bin/env python3

"""
闭集交互规则与标准 JSON 构造函数。
"""

from __future__ import annotations

import re
from datetime import datetime, timedelta, timezone
from typing import Any
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

from .models import IntentResult, IntentType, State


# ---------------------------------------------------------------------
# 文本预处理
# ---------------------------------------------------------------------

_PUNCTUATION_PATTERN = re.compile(
    r"""[
        \s
        ，。！？、；：,.!?;:
        “ ” ‘ ’ "'`
        （）()【】\[\]《》<>
    ]+""",
    re.VERBOSE,
)


def normalize_text(text: str) -> str:
    """
    去除空格和常见标点。

    例如：
        “机器人，请开始服务。”
    转换为：
        “机器人请开始服务”
    """
    return _PUNCTUATION_PATTERN.sub(
        "",
        str(text).strip().lower(),
    )


def contains_any(
    text: str,
    keywords: tuple[str, ...],
) -> bool:
    """判断文本中是否出现任意关键词。"""
    return any(keyword in text for keyword in keywords)


# ---------------------------------------------------------------------
# 任务1：前往交互区I
# ---------------------------------------------------------------------

# 必须同时包含：
# 1. 前往/进入/移动等动作含义；
# 2. 交互区I目标含义。
#
# 使用状态机门控，因此这些关键词只会在
# TASK1_WAIT_COMMAND 状态下生效。
_TASK1_GO_KEYWORDS = (
    "前往",
    "去",
    "到",
    "进入",
    "移动到",
    "导航到",
    "出发去",
)

_TASK1_AREA_KEYWORDS = (
    "交互区一",
    "交互区1",
    "交互区壹",
    "交互区i",
    "交互区Ⅰ",
    "交互区",
    "交汇区",
    "交付区",
    "交互居",
    "交汇居",
    "交付居",
)

# 防止“不要去交互区”之类文本误触发。
_TASK1_NEGATION_KEYWORDS = (
    "不要",
    "别去",
    "不用去",
    "取消",
    "停止",
)


def is_task1_go_interaction_area(
    text: str,
) -> bool:
    """判断是否为任务1前往交互区I指令。"""

    if contains_any(
        text,
        _TASK1_NEGATION_KEYWORDS,
    ):
        return False

    has_go = contains_any(
        text,
        _TASK1_GO_KEYWORDS,
    )

    has_area = contains_any(
        text,
        _TASK1_AREA_KEYWORDS,
    )

    return has_go and has_area


def build_task1_go_interaction_area_result(
) -> IntentResult:
    """构造任务1导航请求。"""
    return IntentResult(
        intent_type=(
            IntentType.TASK1_GO_INTERACTION_AREA
        ),
        slots={
            "target": "interaction_area_1",
            "target_cn": "交互区I",
            "start_navigation": True,
        },
        confidence=1.0,
        next_state=State.TASK1_NAVIGATING,
    )


def build_task1_unknown_result() -> IntentResult:
    """任务1导航指令未命中。"""
    return IntentResult(
        intent_type=IntentType.UNKNOWN,
        slots={
            "reason": "未识别到前往交互区I的任务1指令",
            "reply_text": (
                "我没有听清楚，请再说一遍。"
            ),
        },
        confidence=0.0,
        next_state=State.TASK1_WAIT_COMMAND,
    )


# ---------------------------------------------------------------------
# 时间查询
# ---------------------------------------------------------------------

_TIME_KEYWORDS = (
    "几点",
    "时间",
    "时刻",
)


def is_time_query(text: str) -> bool:
    """判断是否为当前时间查询。"""
    return contains_any(text, _TIME_KEYWORDS)


def _beijing_now() -> datetime:
    """读取 Asia/Shanghai 当前时间。"""
    try:
        tz = ZoneInfo("Asia/Shanghai")
    except ZoneInfoNotFoundError:
        # 极少数裁剪系统可能没有时区数据库。
        tz = timezone(timedelta(hours=8))

    return datetime.now(tz)


def build_time_result() -> IntentResult:
    """构造任务3时间查询结果。"""
    now = _beijing_now()
    hour = int(now.hour)
    minute = int(now.minute)

    return IntentResult(
        intent_type=IntentType.TASK3_TIME_QUERY,
        slots={
            "reply_text": (
                f"现在是北京时间{hour}点{minute}分。"
            ),
            "hour": hour,
            "minute": minute,
            "time_zone": "Asia/Shanghai",
        },
        confidence=1.0,
        next_state=State.INTERACTION_LISTEN,
    )


# ---------------------------------------------------------------------
# 数字颜色识别请求
# ---------------------------------------------------------------------

_DIGIT_KEYWORDS = (
    "数字",
    "号码",
)

_COLOR_KEYWORDS = (
    "颜色",
    "什么色",
)


def is_digit_color_query(text: str) -> bool:
    """
    数字和颜色必须同时出现，降低误判概率。

    支持例如：
        图中的数字是什么颜色
        图片上的数字是什么颜色是什么
        数字是多少，它是什么颜色
    """
    return (
        contains_any(text, _DIGIT_KEYWORDS)
        and contains_any(text, _COLOR_KEYWORDS)
    )


def build_digit_color_result() -> IntentResult:
    """构造数字颜色识别请求。"""
    return IntentResult(
        intent_type=IntentType.TASK3_DIGIT_COLOR_QUERY,
        slots={
            "request": "digit_color_recognition",
        },
        confidence=1.0,
        next_state=State.INTERACTION_LISTEN,
    )


# ---------------------------------------------------------------------
# 表情控制
# ---------------------------------------------------------------------

# 表情匹配规则：
#
# 1. 只要识别文本包含任意一个关键词，就命中该表情；
# 2. 强烈表情必须放在基础表情之前；
# 3. 编号表情必须放在通用表情之前；
# 4. 输出 emotion_id，同时保留中英文名称供日志使用。
#
# 每项格式：
# (
#     关键词集合,
#     内部名称,
#     中文名称,
#     emotion_id,
# )

_EMOJI_RULES: tuple[
    tuple[
        tuple[str, ...],
        str,
        str,
        int,
    ],
    ...,
] = (
    # 必须在“愤怒”之前。
    (
        (
            "加倍愤怒",
            "极度愤怒",
            "暴怒",
        ),
        "extra_angry",
        "加倍愤怒",
        190,
    ),

    # 必须在“崇拜”之前。
    (
        (
            "加倍崇拜",
            "极度崇拜",
        ),
        "extra_admire",
        "加倍崇拜",
        210,
    ),

    # 必须在“开心、快乐”之前。
    (
        (
            "加倍开心",
            "极度开心",
            "超级开心",
        ),
        "extra_happy",
        "加倍开心",
        100,
    ),
    (
        (
            "狂喜",
        ),
        "ecstatic",
        "狂喜",
        101,
    ),

    # 编号卖萌必须放在通用“卖萌”之前。
    (
        (
            "平静-卖萌4",
            "平静卖萌4",
            "卖萌4",
            "卖萌四",
        ),
        "cute_4",
        "平静-卖萌4",
        33,
    ),
    (
        (
            "平静-卖萌3",
            "平静卖萌3",
            "卖萌3",
            "卖萌三",
        ),
        "cute_3",
        "平静-卖萌3",
        32,
    ),
    (
        (
            "平静-卖萌2",
            "平静卖萌2",
            "卖萌2",
            "卖萌二",
        ),
        "cute_2",
        "平静-卖萌2",
        31,
    ),
    (
        (
            "平静-卖萌1",
            "平静卖萌1",
            "卖萌1",
            "卖萌一",
            "卖萌",
        ),
        "cute_1",
        "平静-卖萌1",
        30,
    ),

    (
        (
            "眨眼",
        ),
        "blink",
        "眨眼",
        1,
    ),
    (
        (
            "平静-眼睛变化1",
            "平静眼睛变化1",
            "眼睛变化1",
            "平静眼睛变化一",
            "眼睛变化一",
        ),
        "calm_eye_1",
        "平静-眼睛变化1",
        10,
    ),
    (
        (
            "平静-眼睛变化2",
            "平静眼睛变化2",
            "眼睛变化2",
            "平静眼睛变化二",
            "眼睛变化二",
        ),
        "calm_eye_2",
        "平静-眼睛变化2",
        11,
    ),
    (
        (
            "平静-游戏",
            "平静游戏",
            "游戏表情",
        ),
        "calm_game",
        "平静-游戏",
        20,
    ),
    (
        (
            "闭上眼",
            "闭眼",
        ),
        "close_eyes",
        "闭上眼",
        40,
    ),
    (
        (
            "睁开眼",
            "睁眼",
        ),
        "open_eyes",
        "睁开眼",
        50,
    ),
    (
        (
            "无聊",
        ),
        "bored",
        "无聊",
        60,
    ),
    (
        (
            "异常状态",
            "异常",
        ),
        "abnormal",
        "异常",
        70,
    ),
    (
        (
            "睡着",
            "睡觉",
            "睡眠",
            "困倦",
            "困了",
        ),
        "sleep",
        "睡着",
        80,
    ),
    (
        (
            "快乐",
            "开心",
            "高兴",
            "微笑",
            "笑脸",
            "笑一个",
            "笑一下",
        ),
        "happy",
        "快乐",
        90,
    ),
    (
        (
            "悲伤",
            "伤心",
            "难过",
        ),
        "sad",
        "悲伤",
        110,
    ),
    (
        (
            "同情",
            "关切",
        ),
        "sympathy",
        "同情",
        120,
    ),
    (
        (
            "疑惑",
            "困惑",
            "不解",
        ),
        "confused",
        "疑惑",
        130,
    ),
    (
        (
            "震惊",
            "惊讶",
            "吃惊",
        ),
        "shocked",
        "震惊",
        140,
    ),
    (
        (
            "撒娇",
        ),
        "coquettish",
        "撒娇",
        150,
    ),
    (
        (
            "严肃",
            "认真表情",
        ),
        "serious",
        "严肃",
        160,
    ),
    (
        (
            "思考",
            "思索",
            "想一想",
        ),
        "thinking",
        "思考",
        170,
    ),
    (
        (
            "愤怒",
            "生气",
            "发怒",
        ),
        "angry",
        "愤怒",
        180,
    ),
    (
        (
            "崇拜",
        ),
        "admire",
        "崇拜",
        200,
    ),
    (
        (
            "充电",
            "充能",
        ),
        "charging",
        "充电",
        220,
    ),
)


def match_emoji(
    text: str,
) -> dict[str, Any] | None:
    """根据核心关键词匹配表情ID。"""
    for (
        keywords,
        emoji,
        emoji_cn,
        emotion_id,
    ) in _EMOJI_RULES:
        if contains_any(text, keywords):
            return {
                "emoji": emoji,
                "emoji_cn": emoji_cn,
                "emotion_id": int(emotion_id),
            }

    return None


def build_emoji_result(
    slots: dict[str, Any],
) -> IntentResult:
    """构造表情控制结果。"""
    return IntentResult(
        intent_type=IntentType.TASK3_EMOJI_CONTROL,
        slots=dict(slots),
        confidence=1.0,
        next_state=State.INTERACTION_LISTEN,
    )


# ---------------------------------------------------------------------
# 动作控制
# ---------------------------------------------------------------------

# 动作匹配规则：
#
# 1. 比赛下发的句子一定包含完整动作名称；
# 2. 直接以完整动作名称做子串匹配；
# 3. 同时输出 motion 和 area；
# 4. 不再通过独立方向词进行二次推断；
# 5. 更长的动作名称优先匹配。
#
# 每项格式：
# (
#     完整动作名称,
#     内部动作名称,
#     motion,
#     area,
# )

_ACTION_RULES: tuple[
    tuple[str, str, int, int],
    ...,
] = (
    ("右手挥手", "wave_right", 1002, 2),
    ("左手挥手", "wave_left", 1002, 1),

    ("右手握手", "handshake_right", 1003, 2),
    ("左手握手", "handshake_left", 1003, 1),

    ("右手举手", "raise_hand_right", 1001, 2),
    ("左手举手", "raise_hand_left", 1001, 1),

    ("右手飞吻", "blow_kiss_right", 1004, 2),
    ("左手飞吻", "blow_kiss_left", 1004, 1),

    ("鼓掌", "clap", 3017, 11),

    ("右手敬礼", "salute_right", 1013, 2),
    ("左手敬礼", "salute_left", 1013, 1),

    ("双手比心", "heart_both", 1007, 3),
    ("右手比心", "heart_right", 1007, 2),
    ("左手比心", "heart_left", 1007, 1),

    ("拥抱", "hug", 3008, 11),
    ("加油", "cheer", 3011, 11),

    ("双手平举", "arms_horizontal_both", 1010, 3),
    ("右手平举", "arms_horizontal_right", 1010, 2),
    ("左手平举", "arms_horizontal_left", 1010, 1),

    ("拜拜", "bye", 3031, 11),
    ("动感光波", "dynamic_wave", 3007, 11),

    ("右手击掌", "high_five_right", 1008, 2),
    ("左手击掌", "high_five_left", 1008, 1),

    ("双手打叉", "cross_arms", 3009, 11),

    ("胸前右手挥手", "chest_wave_right", 1011, 2),
    ("胸前左手挥手", "chest_wave_left", 1011, 1),

    ("鞠躬", "bow", 3001, 11),
    ("挠头", "scratch_head", 3024, 11),
    ("抓屁股", "scratch_butt", 3025, 11),
)


def match_action(
    text: str,
) -> dict[str, Any] | None:
    """
    根据完整动作名称匹配motion和area。

    使用长度倒序，防止：
        胸前右手挥手
    被提前识别为：
        右手挥手
    """
    ordered_rules = sorted(
        _ACTION_RULES,
        key=lambda item: len(item[0]),
        reverse=True,
    )

    for (
        action_cn,
        action,
        motion,
        area,
    ) in ordered_rules:
        if action_cn in text:
            return {
                "action": action,
                "action_cn": action_cn,
                "motion": int(motion),
                "area": int(area),
                "reply_text": (
                    f"我正在执行{action_cn}动作。"
                ),
            }

    return None


def build_action_result(
    slots: dict[str, Any],
) -> IntentResult:
    """构造动作控制结果。"""
    return IntentResult(
        intent_type=IntentType.TASK3_ACTION_CONTROL,
        slots=dict(slots),
        confidence=1.0,
        next_state=State.INTERACTION_LISTEN,
    )


# ---------------------------------------------------------------------
# 任务4唤醒
# ---------------------------------------------------------------------

_WAKE_PHRASES = (
    "机器人请开始服务",
    "机器人开始服务",
    "请开始服务",
    "开始服务",
    "机器人帮帮我",
    "机器人帮我一下",
)


def is_task4_wake(text: str) -> bool:
    """判断是否触发任务4自主服务。"""
    return contains_any(text, _WAKE_PHRASES)


def build_task4_wake_result() -> IntentResult:
    """构造任务4第一次输出。"""
    return IntentResult(
        intent_type=IntentType.TASK4_WAKE_SERVICE,
        slots={
            "wake": True,
            "emoji": "concern",
            "emoji_cn": "关切",
            "emotion_id": 120,
            "reply_text": "今天状态怎么样？",
        },
        confidence=1.0,
        next_state=State.TASK4_ASK_STATUS,
    )


# ---------------------------------------------------------------------
# 任务4需求分类
# ---------------------------------------------------------------------

# 头部不适必须同时满足：
# 1. 出现头部相关词；
# 2. 出现疼痛、不适或眩晕等症状词。
_HEAD_PART_KEYWORDS = (
    "头",
    "头部",
    "脑袋",
    "脑子",
)

_HEAD_SYMPTOM_KEYWORDS = (
    "疼",
    "痛",
    "不舒服",
    "不适",
    "难受",
    "晕",
    "眩晕",
)

# 口渴使用明确的核心关键词。
_THIRSTY_KEYWORDS = (
    "渴",
    "口干",
    "喝水",
    "喝点水",
    "想喝水",
    "想喝点水",
    "要喝水",
    "要水",
)

# 饥饿使用核心字“饿”。
#
# 即使ASR识别成：
#   激饿
#   鸡饿
#   微饿
# 只要仍包含“饿”，就可以正确分类。
_HUNGRY_KEYWORDS = (
    "饿",
    "饥饿",
    "吃饭",
    "想吃饭",
    "吃东西",
    "想吃东西",
    "想吃点东西",
    "拿点吃的",
)


_TASK4_NEED_SLOTS = {
    "head_uncomfortable": {
        "need": "head_uncomfortable",
        "target_object": "medicine_box",
        "reply_text": "听起来不太舒服，我去帮您拿药。",
        "start_autonomous_service": True,
        "execution_order": [
            "speak_response",
            "autonomous_service",
        ],
    },
    "thirsty": {
        "need": "thirsty",
        "target_object": "cup",
        "reply_text": "好的，我去帮您拿杯水。",
        "start_autonomous_service": True,
        "execution_order": [
            "speak_response",
            "autonomous_service",
        ],
    },
    "hungry": {
        "need": "hungry",
        "target_object": "bread",
        "reply_text": "好的，我去帮您拿点吃的。",
        "start_autonomous_service": True,
        "execution_order": [
            "speak_response",
            "autonomous_service",
        ],
    },
}


def match_task4_need(
    text: str,
) -> dict[str, Any] | None:
    """
    匹配任务4的三类需求。

    规则：
    1. 头部不适必须同时包含头部词和症状词；
    2. 口渴只要包含明确口渴关键词即可；
    3. 饥饿只要包含“饿”等核心关键词即可；
    4. 没有命中或者同时命中多类时返回None。
    """

    has_head_part = contains_any(
        text,
        _HEAD_PART_KEYWORDS,
    )

    has_head_symptom = contains_any(
        text,
        _HEAD_SYMPTOM_KEYWORDS,
    )

    has_head_uncomfortable = (
        has_head_part
        and has_head_symptom
    )

    has_thirsty = contains_any(
        text,
        _THIRSTY_KEYWORDS,
    )

    has_hungry = contains_any(
        text,
        _HUNGRY_KEYWORDS,
    )

    matched_needs: list[str] = []

    if has_head_uncomfortable:
        matched_needs.append(
            "head_uncomfortable"
        )

    if has_thirsty:
        matched_needs.append(
            "thirsty"
        )

    if has_hungry:
        matched_needs.append(
            "hungry"
        )

    # 未识别到需求，或者一句话同时包含多个需求，
    # 都不擅自选择。
    if len(matched_needs) != 1:
        return None

    need_name = matched_needs[0]

    return dict(
        _TASK4_NEED_SLOTS[need_name]
    )


def build_task4_need_result(
    slots: dict[str, Any],
) -> IntentResult:
    """构造任务4第二次输出。"""
    return IntentResult(
        intent_type=IntentType.TASK4_NEED_CLASSIFICATION,
        slots=dict(slots),
        confidence=1.0,
        next_state=State.TASK4_WAIT_SERVICE_COMPLETE,
    )


# ---------------------------------------------------------------------
# unknown
# ---------------------------------------------------------------------

def build_task3_unknown_result() -> IntentResult:
    """任务3或任务4唤醒未命中。"""
    return IntentResult(
        intent_type=IntentType.UNKNOWN,
        slots={
            "reason": "未命中任务3指令或任务4唤醒规则",
            "reply_text": "我没有听清楚，请再说一遍。",
        },
        confidence=0.0,
        next_state=State.INTERACTION_LISTEN,
    )


def build_task4_unknown_result() -> IntentResult:
    """任务4需求未命中。"""
    return IntentResult(
        intent_type=IntentType.UNKNOWN,
        slots={
            "reason": "未识别到头部不适、口渴或饥饿需求",
            "reply_text": (
                "我没有听清楚您的需求，请再说一遍。"
            ),
        },
        confidence=0.0,
        next_state=State.TASK4_LISTEN_NEED,
    )
