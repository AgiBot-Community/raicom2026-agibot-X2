# 🤖 star_agibot — Agibot X2 竞赛机器人全流程系统

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2: Humble](https://img.shields.io/badge/ROS_2-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu: 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420.svg)](https://releases.ubuntu.com/22.04/)
[![Python: 3.10](https://img.shields.io/badge/Python-3.10-3776AB.svg)](https://www.python.org/)

## 📖 项目简介

`star_agibot` 是基于 **Agibot X2 / AimDK + ROS 2 Humble** 搭建的竞赛机器人完整任务系统，覆盖：

- 语音采集与离线 ASR；
- 本地闭集意图识别与交互状态机；
- 任务总调度；
- 官方地图下的双导航执行；
- 数字/颜色与物体视觉识别；
- 离线 TTS、表情与预设动作；
- RGB-D 三维坐标解算；
- IK 逆解、机械臂与夹爪抓取；
- 任务间麦克风、导航、视觉和抓取时序管理。

项目采用**分层 + ROS 2 Topic/Service 解耦**设计。自然语言不会直接控制机器人，而是先由 `interaction_agent` 转换为白名单 JSON 意图，再交由 `x2_execution_pkg/coordinator_node` 调度具体能力节点。

```text
用户语音
   │
   ▼
voice_asr
   │ /ai_agent/recognized_text
   ▼
interaction_agent
   │ /ai_agent/input_json
   ▼
coordinator_node
   ├────────────► voice_node ─────────► 扬声器 / 表情 / 动作
   ├────────────► vision_node ────────► /competition/grasp_target
   ├────────────► task1_pose_nav ─────► /cmd_vel
   └────────────► service_pose_nav ───► /cmd_vel
                                            │
                                            ▼
                                      cmd_vel_bridge
                                            │
                                            ▼
                               /aima/mc/locomotion/velocity

/competition/grasp_target
   │
   ▼
x2_grasp_auto_trigger
   │
   ▼
x2_grasp_executor_server
   │
   ▼
x2_ik_sdk + omnipicker_hand
   │
   ▼
机械臂 / 夹爪
```


> **任务编号说明**  
> 当前比赛规则中的部分“任务 2”逻辑，在历史代码中仍沿用 `task3_*` 命名。为避免赛前大范围改名破坏已联调协议，本项目保留代码命名，README 按实际比赛流程解释。

---

# 📂 一、项目结构

根据当前工作空间整理，推荐提交时保持如下结构：

```text
star_agibot/
├── README.md                         # 项目总说明
├── LICENSE                           # MIT License（项目原创代码）
├── .gitignore                        # 敏感信息 / 构建产物排除规则
│
├── models/                           # 推荐统一存放视觉模型
├── vits-zh-hf-fanchen-C/             # 本地 TTS 模型/资源（如实际使用）
│
├── agibot_arm64_wheels/              # ARM64 基础离线 Python wheels
├── agibot_yolo_wheels/               # ARM64 YOLO / ONNX 离线 wheels
├── runtime/                           # 运行时资源
├── scripts/                           # 环境激活、辅助运行脚本
├── final_scripts/                     # 比赛现场最终辅助脚本
├── maps/                              # 地图/现场辅助文件（如需要）
├── audio_channel_test/                # 麦克风通道测试工具
│
├── check_external_mic.py              # 外置麦克风检查
├── send_listen_true.py                # 首轮开麦辅助脚本
├── send_pose.py                       # 位姿调试工具
├── test_gray_distribution.py          # 视觉灰度调试工具
│
└── src/
    ├── voice_asr/                     # 🎙️ 音频采集、VAD、离线 ASR
    ├── interaction_agent/             # 🧠 闭集意图 + 状态机
    ├── x2_execution_pkg/              # 🎛️ Coordinator / Vision / Voice
    ├── race_task/                     # 🧭 定位桥、双导航、速度桥
    ├── grasping/                      # 🦾 抓取守护、服务端、触发器
    ├── x2_ik_sdk/                     # 🧮 IK SDK / 离线依赖
    ├── map_tf_distribution/           # 🗺️ 地图/TF 支撑包
    ├── py_examples/                   # 🧪 AimDK 示例与复位工具
    └── ruckig/                        # 第三方轨迹相关依赖
```

## 1.1 核心包职责

| 包 | 主要职责 |
| --- | --- |
| `voice_asr` | 内/外置麦克风音频接入、VAD、SenseVoice/离线 ASR，发布识别文本 |
| `interaction_agent` | 状态约束下的闭集语义识别，将文本变为统一 JSON 意图 |
| `x2_execution_pkg` | 总调度、视觉、离线语音播报、表情/动作以及任务时序 |
| `race_task` | 官方地图定位桥、任务 1 导航、服务导航、底盘速度桥 |
| `grasping` | 夹爪守护、抓取 Executor、目标抓取触发 |
| `x2_ik_sdk` | IK 逆运动学 SDK 与离线依赖 |
| `map_tf_distribution` | 定位/TF/地图辅助能力 |
| `py_examples` | 机器人模式、姿态复位和官方接口测试 |
| `ruckig` | 第三方轨迹规划依赖（按上游许可证使用） |

---

# 🧩 二、核心模块说明

## 2.1 `voice_asr`

```text
voice_asr/
├── config/                              # ASR 三种运行模式配置
│   ├── voice_asr_raw_internal.yaml      # 内置麦 + Raw 音频 + Silero VAD
│   ├── voice_asr_raw_external.yaml      # 外置麦 + Raw 音频 + Silero VAD
│   └── voice_asr_processed_external.yaml # 外置麦 + Processed 音频 + AimDK VAD
├── resource/                            # ROS 2 ament 包索引资源
├── test/                                # ROS 2 / Python 标准测试
├── voice_asr/                           # 核心语音识别代码
│   ├── __init__.py                      # Python 包初始化
│   ├── voice_asr_node.py                # ASR 主节点与监听控制
│   ├── asr_engine.py                    # 本地离线 ASR 推理
│   ├── vad_engine.py                    # Silero VAD 语音活动检测
│   ├── listening_session.py             # Raw 音频监听与分段管理
│   ├── robot_audio_source.py            # AimDK Raw 音频接入
│   ├── external_mic.py                  # 外置麦克风切换与确认
│   ├── processed_audio_source.py        # AimDK Processed 音频接入
│   └── processed_listening_session.py   # Processed 音频监听会话管理
├── LICENSE                              # 包级开源协议
├── README.md                            # voice_asr 包说明
├── package.xml                          # ROS 2 包描述与依赖
├── setup.cfg                            # Python ROS 2 安装配置
└── setup.py                             # Python 包及可执行入口配置
```



比赛时三个入口**三选一，禁止同时运行多个 ASR 实例**：

```text
voice_asr_raw_internal
    └── 内置麦 + 原始音频 + 本地 Silero VAD

voice_asr_raw_external
    └── 外置麦 + 原始音频 + 本地 Silero VAD

voice_asr_processed_external
    └── 外置麦 + AimDK 处理后音频 + AimDK VAD
```

统一输出：

```text
/ai_agent/recognized_text
std_msgs/msg/String
```
**目前前两个入口已在真实机器人上测试成功，最后一个入口未测试成功，仍需微调。**

## 2.2 `interaction_agent`

```text
interaction_agent/
├── interaction_agent/               # 🧠 交互核心逻辑
│   ├── __init__.py
│   ├── models.py                    # 状态、意图枚举与标准 IntentResult
│   ├── rules.py                     # 闭集关键词、槽位、表情/动作/需求规则
│   ├── intent_dispatcher.py         # 按当前状态进行意图白名单分流
│   ├── state_machine.py             # 比赛交互有限状态机
│   └── interaction_node.py          # ROS 2 节点：文本输入 / JSON 输出 / 状态同步
├── resource/
│   └── interaction_agent
├── test/                            # ROS 2 / Python 标准测试目录
├── package.xml
├── setup.cfg
└── setup.py
```

核心数据流：

```text
/ai_agent/recognized_text
        ↓
interaction_node
        ↓
rules + intent_dispatcher + state_machine
        ↓
/ai_agent/input_json
```

统一 JSON 示例：

```json
{
  "intent_type": "task1_go_interaction_area",
  "slots": {
    "target": "interaction_area_1"
  },
  "confidence": 1.0,
  "next_state": "TASK1_NAVIGATING"
}
```

主要意图：

| 阶段 | `intent_type` |
| --- | --- |
| 任务 1 | `task1_go_interaction_area` |
| 基础交互 | `task3_time_query` |
| 基础交互 | `task3_digit_color_query` |
| 基础交互 | `task3_emoji_control` |
| 基础交互 | `task3_action_control` |
| 自主服务 | `task4_wake_service` |
| 自主服务 | `task4_need_classification` |
| 兜底 | `unknown` |

状态机核心：

```text
TASK1_WAIT_COMMAND
        │
        ▼
TASK1_NAVIGATING
        │ task1 reached + listen_control=True
        ▼
INTERACTION_LISTEN
        │
        ├── 时间 / 数字颜色 / 表情 / 动作
        │
        └── 唤醒自主服务
                 ▼
          TASK4_ASK_STATUS
                 ▼
          TASK4_LISTEN_NEED
                 ▼
   TASK4_WAIT_SERVICE_COMPLETE
                 ▼
               FINISH
```

## 2.3 `x2_execution_pkg`

```text
x2_execution_pkg/
├── config/                   # 🌟 核心配置文件目录
│   ├── coordinator_nav.yaml  # 导航与总指挥参数配置
│   └── vision_config.yaml    # 视觉模型路径脱敏配置
├── models/                   # 🧠 本地离线模型存放区
│   ├── best.onnx             # 夹取识别特训模型 (Plan A)
│   ├── yoloe_task4.onnx      # 夹取识别开放提示词大模型 (Plan B)
│   ├── yolo.onnx             # 数字识别 ONNX 模型
│   └── yolo_digits.pt        # 数字识别 PT 原模型
├── resource/
├── test/                     # ROS 2 标准测试目录
├── x2_execution_pkg/         # 🚀 核心代码逻辑
│   ├── __init__.py
│   ├── coordinator_node.py   # 总指挥大脑节点
│   ├── vision_node.py        # 视觉感知节点
│   └── voice_node.py         # 语音交互节点
├── package.xml
├── setup.cfg
└── setup.py

```

- `coordinator_node`：订阅意图 JSON，控制麦克风，选择任务 1/服务导航，调度视觉、播报、动作与抓取。
- `vision_node`：数字/颜色识别、目标物体识别、RGB-D 三维解算、TF 转换。
- `voice_node`：本地 TTS、音频下发、播报结束后的麦克风策略。

内部接口：

```text
/system/voice_cmd
/system/vision_cmd
/system/vision_result
```

## 2.4 `race_task`

```text
race_task/
├── config/                           # 🌟 真机导航参数
│   ├── competition_official_voice_nav.yaml
│   └── service_official_map_nav.yaml
├── launch/
│   ├── task1_app_ready.launch.py    # 任务1：TF + 速度桥 + task1 导航
│   └── dual_navigation.launch.py    # 可选：完整比赛同时拉起两套导航
├── race_task/                        # 🚀 导航与底盘核心代码
│   ├── __init__.py
│   ├── tf_pose_bridge.py            # map->base_link TF 转PoseStamped
│   ├── task1_pose_nav.py            # 出发区 → 交互区 I
│   ├── service_pose_nav.py          # 交互区 → 作业区/服务区域
│   └── cmd_vel_bridge.py            # /cmd_vel → AimDK 底盘速度接口
├── resource/
│   └── race_task
├── test/
├── package.xml
├── setup.cfg
└── setup.py
```

正式链路：

```text
map -> base_link TF
        ↓
tf_pose_bridge
        ↓
/map_tf_distribution/localization_pose
        ↓
task1_pose_nav / service_pose_nav
        ↓
/cmd_vel
        ↓
cmd_vel_bridge
        ↓
/aima/mc/locomotion/velocity
```

任务 1 与服务导航**必须使用不同 Goal/Status Topic**：

```text
Task1:
  /race_task/task1/goal_pose
  /race_task/task1/nav_status

Service:
  /race_task/service/goal_pose
  /race_task/service/nav_status
```

两个导航节点可同时启动，但只有收到各自目标后才允许进入控制状态；任何时刻都应避免两个导航器同时发布有效运动控制。

## 2.5 `grasping` + `x2_ik_sdk`
模块层级结构

```text
star_agibot/
├── src/
│   ├── grasping/                  # 核心抓取功能包 (包含夹爪控制、服务端、触发器)
│   ├── x2_ik_sdk/                 # 逆运动学 (IK) 离线依赖与 SDK
│   │   ├── offline_deps/          # 离线安装包 (numpy, pin等)
│   │   └── src/                   # SDK 源码
│   └── py_examples/               # 测试脚本与复位用例 (set_mc_action)
```


核心组件：

```text
omnipicker_hand
        └── 夹爪底层守护/状态

x2_grasp_executor_server
        └── 接收抓取请求、IK 解算、机械臂轨迹执行

x2_grasp_auto_trigger
        └── 监听 /competition/grasp_target 并触发 Executor
```

抓取目标输入：

```text
/competition/grasp_target
std_msgs/msg/String
```

示例：

```json
{
  "object_name": "cup",
  "target_point": [0.38, 0.0, 0.20]
}
```

`target_point` 应为执行链约定的三维坐标；当前视觉执行链按 `base_link` 坐标系对接。

---

# 🛠️ 三、环境要求

## 3.1 系统

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10
- Agibot X2 / AimDK
- `~/aimdk` 已正确安装并可 `source ~/aimdk/install/setup.bash`
- 真机推荐 NVIDIA Jetson Orin NX / ARM64
- RGB-D 相机、麦克风、扬声器
- 官方 App 地图及重定位能力

所有普通 ROS 终端统一按以下顺序加载环境：

```bash
cd ~/star_agibot

source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
```

如果仓库中的 `scripts/activate_robot_runtime.sh` 用于加载 ASR 离线运行库，可在上述三行之后追加：

```bash
source ~/star_agibot/scripts/activate_robot_runtime.sh
```

> 不要在不同机器人镜像之间盲目硬编码 `RMW_IMPLEMENTATION`。比赛机优先使用其系统默认 RMW；只有确认 ABI 兼容性后再显式指定。

## 3.2 视觉 / TTS Python 依赖

项目已验证/约定的关键版本：

```text
numpy==1.24.3
opencv-python-headless==4.8.1.78
torch==2.1.0
torchvision==0.16.0
sherpa-onnx==1.13.4
```

在线开发环境：

```bash
pip install numpy==1.24.3 opencv-python-headless==4.8.1.78
pip install torch==2.1.0 torchvision==0.16.0
pip install sherpa-onnx==1.13.4 ultralytics onnx onnxruntime
pip install soundfile PyYAML Shapely pyclipper Pillow six
```

## 3.3 ARM64 离线安装

仓库根目录已保留：

```text
agibot_arm64_wheels/
agibot_yolo_wheels/
```

机器人断网时：

```bash
cd ~/star_agibot/agibot_arm64_wheels

pip install --no-index --find-links=. --no-deps \
  numpy==1.24.3 \
  opencv-python-headless==4.8.1.78

pip install --no-index --find-links=. \
  numpy==1.24.3 \
  opencv-python-headless==4.8.1.78 \
  sherpa-onnx==1.13.4 \
  soundfile PyYAML Shapely onnxruntime pyclipper Pillow six
```

YOLO / ONNX：

```bash
cd ~/star_agibot/agibot_yolo_wheels

pip install --no-index --find-links=. \
  torch==2.1.0 \
  torchvision==0.16.0 \
  onnx onnxruntime \
  numpy==1.24.3 \
  opencv-python-headless==4.8.1.78

# 文件名以目录中的实际版本为准
pip install --no-index --no-deps ./ultralytics-*.whl
```

---

#  四、IK 抓取独立运行环境

机械臂 IK 使用独立 Python venv，避免与系统 ROS / 视觉依赖互相污染。

```bash
deactivate 2>/dev/null || true
unset PYTHONPATH
export PYTHONNOUSERSITE=1

rm -rf ~/.venvs/x2-ik-runtime
python3 -m venv ~/.venvs/x2-ik-runtime
source ~/.venvs/x2-ik-runtime/bin/activate

cd ~/star_agibot/src/x2_ik_sdk

python3 -m pip install --no-index --find-links=offline_deps \
  numpy pin setuptools wheel

python3 -m pip install --no-index --find-links=offline_deps \
  x2_ik_sdk
```

> 只有抓取相关终端需要激活 `x2-ik-runtime`。ASR、导航、Coordinator、Vision、Voice 等普通 ROS 终端不要误激活该 venv。

---

# 五、编译

## 5.1 编译普通 ROS 包

新终端执行：

```bash
cd ~/star_agibot

source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash

unset RMW_IMPLEMENTATION

colcon build \
  --symlink-install \
  --packages-select \
  voice_asr \
  interaction_agent \
  race_task \
  x2_execution_pkg \
  py_examples
```

完成后：

```bash
source ~/star_agibot/install/setup.bash
```

检查：

```bash
ros2 pkg prefix voice_asr
ros2 pkg prefix interaction_agent
ros2 pkg prefix race_task
ros2 pkg prefix x2_execution_pkg
```

## 5.2 编译抓取包

```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2-ik-runtime/bin/activate
export PYTHONNOUSERSITE=1

cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash

colcon build \
  --symlink-install \
  --packages-select grasping
```

完成后：

```bash
source ~/star_agibot/install/setup.bash
```

## 5.3 可执行入口检查

```bash
ros2 pkg executables voice_asr
ros2 pkg executables interaction_agent
ros2 pkg executables race_task
ros2 pkg executables x2_execution_pkg
ros2 pkg executables grasping
```

至少应确认以下核心入口存在：

```text
interaction_agent interaction_node

race_task tf_pose_bridge
race_task task1_pose_nav
race_task service_pose_nav
race_task cmd_vel_bridge

x2_execution_pkg coordinator_node
x2_execution_pkg vision_node
x2_execution_pkg voice_node

grasping omnipicker_hand
grasping x2_grasp_executor_server
grasping x2_grasp_auto_trigger
```

---

# 六、真机启动前检查

## 6.1 AimDK

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash

ros2 pkg prefix aimdk_msgs
```

应指向当前机器人正确的 AimDK 安装环境。

## 6.2 官方地图与定位

先确认地图：

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 topic info /map -v
```

确认 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

确认导航使用的统一定位：

```bash
ros2 topic echo \
  /map_tf_distribution/localization_pose \
  --once
```

如果该 Topic 没有实时数据，**不要发送真实导航目标**。

## 6.3 双导航接口

```bash
ros2 topic info /race_task/task1/goal_pose --verbose
ros2 topic info /race_task/task1/nav_status --verbose

ros2 topic info /race_task/service/goal_pose --verbose
ros2 topic info /race_task/service/nav_status --verbose
```

任务 1 预期：

```text
/race_task/task1/goal_pose
Publisher: /coordinator_node
Subscriber: /task1_pose_nav
```

## 6.4 ASR 单实例检查

```bash
pgrep -af \
'voice_asr_node|voice_asr_raw_internal|voice_asr_raw_external|voice_asr_processed_external'
```

比赛时只能有一个实际 ASR 实例。

## 6.5 外置麦检查（可选）

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash

unset RMW_IMPLEMENTATION

python3 ~/star_agibot/check_external_mic.py
```

---

# 七、完整跑通步骤

下面按“基础能力先启动，Coordinator 最后启动”的顺序执行。

## 7.1 终端 A：夹爪守护

```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2-ik-runtime/bin/activate
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"

ros2 run grasping omnipicker_hand --publish close right
```

## 7.2 终端 B：抓取 Executor

```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2-ik-runtime/bin/activate
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"

ros2 run grasping x2_grasp_executor_server
```

## 7.3 终端 C：抓取自动触发器

```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2-ik-runtime/bin/activate
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"

ros2 run grasping x2_grasp_auto_trigger
```

## 7.4 终端 D：任务 1 导航链

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 launch race_task \
  task1_app_ready.launch.py \
  params_file:=$HOME/star_agibot/src/race_task/config/competition_official_voice_nav.yaml
```

该 launch 应包含：

```text
tf_pose_bridge
cmd_vel_bridge
task1_pose_nav
```

`competition_official_voice_nav.yaml` 中最终接口应为：

```yaml
goal_pose_topic: "/race_task/task1/goal_pose"
nav_status_topic: "/race_task/task1/nav_status"
wait_for_goal_from_rviz: true
```

这里 `wait_for_goal_from_rviz: true` 的历史名称实际表示“等待外部 Goal”，正式发布者是 Coordinator。

## 7.5 终端 E：服务导航

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 run race_task service_pose_nav \
  --ros-args \
  --params-file \
  ~/star_agibot/src/race_task/config/service_official_map_nav.yaml
```

参数文件应对应：

```yaml
service_pose_nav:
  ros__parameters:
    pose_topic: "/map_tf_distribution/localization_pose"
    cmd_topic: "/cmd_vel"
    goal_pose_topic: "/race_task/service/goal_pose"
    nav_status_topic: "/race_task/service/nav_status"
    wait_for_goal_from_rviz: true
```

其余控制频率、速度上限、XY/Yaw 容差以真机最终标定值为准。

## 7.6 终端 F：交互意图节点

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 run interaction_agent interaction_node
```

比赛起始状态应为：

```text
TASK1_WAIT_COMMAND
```

## 7.7 终端 G：ASR（三选一）

### 推荐候选 1：外置麦 raw

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

unset RMW_IMPLEMENTATION

ros2 run voice_asr \
  voice_asr_raw_external \
  --ros-args \
  --params-file \
  ~/star_agibot/src/voice_asr/config/voice_asr_raw_external.yaml
```

### 候选 2：外置麦 processed（真实机器人未测试成功）

```bash
ros2 run voice_asr \
  voice_asr_processed_external \
  --ros-args \
  --params-file \
  ~/star_agibot/src/voice_asr/config/voice_asr_processed_external.yaml
```

### 备用：内置麦

```bash
ros2 run voice_asr \
  voice_asr_raw_internal \
  --ros-args \
  --params-file \
  ~/star_agibot/src/voice_asr/config/voice_asr_raw_internal.yaml
```

**三者只能启动一个。**

## 7.8 终端 H：执行层 Voice

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 run x2_execution_pkg voice_node
```

## 7.9 终端 I：执行层 Vision

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 run x2_execution_pkg vision_node \
  --ros-args \
  --params-file \
  ~/star_agibot/src/x2_execution_pkg/config/vision_config.yaml
```

## 7.10 终端 J：Coordinator（最后启动）

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

ros2 run x2_execution_pkg coordinator_node \
  --ros-args \
  --params-file \
  ~/star_agibot/src/x2_execution_pkg/config/coordinator_nav.yaml
```

Coordinator 启动后，再次确认：

```bash
ros2 node info /coordinator_node
ros2 node info /task1_pose_nav
ros2 node info /service_pose_nav
```

## 7.11 开始第一轮监听

确认所有节点正常、定位正常、机器人周围安全后：

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash

python3 ~/star_agibot/send_listen_true.py
```

然后说：

```text
请前往交互区一
```

---

# 八、比赛主流程

## 8.1 任务 1 → 基础交互

```text
“请前往交互区一”
        ↓
voice_asr
        ↓
/ai_agent/recognized_text
        ↓
interaction_agent
        ↓
task1_go_interaction_area
        ↓
/ai_agent/input_json
        ↓
coordinator_node
        ├── 关闭麦克风
        └── /race_task/task1/goal_pose
                    ↓
             task1_pose_nav
                    ↓
                 /cmd_vel
                    ↓
              机器人移动
                    ↓
/race_task/task1/nav_status = "reached"
                    ↓
             coordinator_node
                    ↓
/ai_agent/listen_control = True
                    ↓
interaction_agent:
TASK1_NAVIGATING → INTERACTION_LISTEN
                    ↓
基础交互开始
```

## 8.2 自主服务

```text
用户唤醒 / 描述需求
        ↓
interaction_agent
        ↓
task4_need_classification
        ↓
coordinator_node
        ├── TTS 回复
        ├── 关闭麦克风
        └── /race_task/service/goal_pose
                    ↓
             service_pose_nav
                    ↓
/race_task/service/nav_status = "reached"
                    ↓
             coordinator_node
                    ↓
            /system/vision_cmd
                    ↓
               vision_node
                    ↓
        /competition/grasp_target
                    ↓
         x2_grasp_auto_trigger
                    ↓
      x2_grasp_executor_server
                    ↓
         IK + 机械臂 + 夹爪
```

---

# 九、ROS 2 接口汇总

| 方向 | Topic / Service | 类型 | 用途 |
| --- | --- | --- | --- |
| ASR → Agent | `/ai_agent/recognized_text` | `std_msgs/msg/String` | 识别文本 |
| Agent → Coordinator | `/ai_agent/input_json` | `std_msgs/msg/String` | 标准意图 JSON |
| Coordinator → ASR/Agent | `/ai_agent/listen_control` | `std_msgs/msg/Bool` | 开关麦与状态同步 |
| Coordinator → Voice | `/system/voice_cmd` | `std_msgs/msg/String` | 播报命令 JSON |
| Coordinator → Vision | `/system/vision_cmd` | `std_msgs/msg/String` | 视觉任务 JSON |
| Vision → Coordinator | `/system/vision_result` | `std_msgs/msg/String` | 视觉结果 JSON |
| Coordinator → Task1 Nav | `/race_task/task1/goal_pose` | `geometry_msgs/msg/PoseStamped` | 任务 1 目标 |
| Task1 Nav → Coordinator | `/race_task/task1/nav_status` | `std_msgs/msg/String` | `"reached"` |
| Coordinator → Service Nav | `/race_task/service/goal_pose` | `geometry_msgs/msg/PoseStamped` | 服务目标 |
| Service Nav → Coordinator | `/race_task/service/nav_status` | `std_msgs/msg/String` | `"reached"` |
| TF Bridge → Nav | `/map_tf_distribution/localization_pose` | `geometry_msgs/msg/PoseStamped` | 实时地图位姿 |
| Nav → Speed Bridge | `/cmd_vel` | `geometry_msgs/msg/Twist` | 导航速度 |
| Speed Bridge → AimDK | `/aima/mc/locomotion/velocity` | AimDK 消息 | 真机底盘速度 |
| Vision → Grasp | `/competition/grasp_target` | `std_msgs/msg/String` | 3D 抓取目标 |
| Vision input | `/aima/hal/sensor/rgbd_head_front/rgb_image` | `sensor_msgs/msg/Image` | RGB |
| Vision input | `/aima/hal/sensor/rgbd_head_front/depth_image` | `sensor_msgs/msg/Image` | Depth |
| Vision input | `/aima/hal/sensor/rgbd_head_front/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| Voice → Audio | `/aima/hal/audio/playback` | `aimdk_msgs/msg/AudioPlayback` | 音频播放 |
| Coordinator → Face | `PlayEmoji` 相关 AimDK Service | AimDK Service | 表情 |
| Coordinator → Motion | `SetMcPresetMotion` 相关 AimDK Service | AimDK Service | 预设动作 |

---

# 十、分模块联调

## 10.1 只测试 Agent，不让机器人运动

只启动 `interaction_agent`，然后：

```bash
ros2 topic echo /ai_agent/input_json
```

模拟 ASR：

```bash
ros2 topic pub --once \
  /ai_agent/recognized_text \
  std_msgs/msg/String \
  "{data: '请前往交互区一'}"
```

应看到：

```text
task1_go_interaction_area
```

## 10.2 模拟任务 1 到达

```bash
ros2 topic pub --once \
  /ai_agent/listen_control \
  std_msgs/msg/Bool \
  "{data: true}"
```

状态应由：

```text
TASK1_NAVIGATING
```

切换至：

```text
INTERACTION_LISTEN
```

## 10.3 抓取链连通测试

**确认机械臂周围无人、姿态安全后**：

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once \
  /competition/grasp_target \
  std_msgs/msg/String \
  "data: '{\"object_name\":\"test_cup\",\"target_point\":[0.38,0.0,0.20]}'"
```

## 10.4 机械臂复位

```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2-ik-runtime/bin/activate
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"

cd ~/star_agibot/src/py_examples
python3 -m py_examples.set_mc_action SD
```

---

# 十一、安全约定

1. **启动导航节点本身不应让机器人立即运动。** 导航器必须等待外部 Goal。
2. 下达真实导航目标前必须确认 `/map_tf_distribution/localization_pose` 实时有效。
3. Task1 和 Service 两个导航器不得同时处于有效控制状态。
4. 导航结束、异常、超时、节点退出时都应发送零速度。
5. ASR 同一时间只能运行一个实例，防止重复识别与重复 JSON。
6. 机器人播报和底盘运动阶段合理关闭麦克风，避免自激和电机噪声误触发。
7. 机械臂/夹爪测试前清空工作空间，优先使用低速和安全姿态。
8. 调试 `/competition/grasp_target` 时不要使用未经确认的三维坐标。
9. 不要将云端模型输出直接映射成任意运动指令；只接受本地白名单意图。
10. 比赛机更换系统镜像后，先验证 AimDK、RMW、Fast-CDR 与消息 ABI，再启动上层节点。

---

# 十二、常见问题

### 12.1 `task1_pose_nav` 收不到目标

```bash
ros2 topic info /race_task/task1/goal_pose --verbose
```

应同时看到：

```text
/coordinator_node
/task1_pose_nav
```

检查 YAML 是否仍使用旧的：

```text
/race_task/nav_goal
/goal_pose
```

最终项目应统一到：

```text
/race_task/task1/goal_pose
```

### 12.2 定位没有数据

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /map_tf_distribution/localization_pose --once
```

先修复官方 App 重定位 / TF，再测试真机导航。

### 12.3 一句话被处理两次

检查是否启动了多个 ASR：

```bash
pgrep -af \
'voice_asr_node|voice_asr_raw_internal|voice_asr_raw_external|voice_asr_processed_external'
```

只保留一个。

### 12.4 Python/IK 依赖互相污染

抓取终端使用：

```bash
~/.venvs/x2-ik-runtime
```

普通 ROS 节点不要激活该 venv。

### 12.5 模型路径找不到

不要在源码里硬编码绝对路径。检查：

```bash
~/star_agibot/src/x2_execution_pkg/config/vision_config.yaml
```

并将模型实际路径指向本机的 `~/star_agibot/models/` 或项目当前模型目录。

---


# 十三、License 与第三方组件

本仓库的**项目原创代码**采用 [MIT License](LICENSE)。

以下内容可能来自第三方，**不因项目根目录使用 MIT 而改变其原许可证**：

- Agibot AimDK / `aimdk_msgs`；
- `x2_ik_sdk` 及其二进制/离线依赖；
- `ruckig`；
- ONNX / PyTorch / Ultralytics / Sherpa-ONNX；
- 预训练模型、字体、语音模型和数据集；
- 机器人厂商提供的地图、SDK、资源文件。

发布或再分发前请分别检查其上游许可证和比赛方授权要求。

---
