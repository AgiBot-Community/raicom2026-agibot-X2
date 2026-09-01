# 🤖 star_agibot — Agibot X2 Competition Robot Full‑Workflow System
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2: Humble](https://img.shields.io/badge/ROS_2-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu: 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420.svg)](https://releases.ubuntu.com/22.04/)
[![Python: 3.10](https://img.shields.io/badge/Python-3.10-3776AB.svg)](https://www.python.org/)

## 📖 Project Introduction
`star_agibot` is a complete competition‑task robot system built on **Agibot X2 / AimDK + ROS 2 Humble**. It covers:
- Voice acquisition and offline ASR;
- Local closed‑set intent recognition and interaction state machine;
- Global task scheduling;
- Dual‑navigation execution under official maps;
- Digit / color and object visual recognition;
- Offline TTS, facial expressions and preset motions;
- RGB‑D 3D coordinate solving;
- IK inverse kinematics, robotic arm and gripper grasping;
- Timing management for microphones, navigation, vision and grasping across tasks.

This project adopts a **layered + ROS 2 Topic/Service decoupled** design. Natural language does not directly control the robot. Instead, `interaction_agent` converts user input into whitelisted JSON intents, which are then passed to `x2_execution_pkg/coordinator_node` to schedule specific capability nodes.

```text
User Voice
   │
   ▼
voice_asr
   │ /ai_agent/recognized_text
   ▼
interaction_agent
   │ /ai_agent/input_json
   ▼
coordinator_node
   ├────────────► voice_node ─────────► Speaker / Facial Expressions / Motions
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
Robotic Arm / Gripper
```

> **Task Number Note**
> Portions of the current competition‑rule “Task 2” logic still use `task3_*` naming in legacy code. To avoid breaking already‑integrated protocols through large‑scale pre‑competition renaming, the original code naming is preserved. The README explains behavior following the actual competition workflow.

---

# 📂 I. Project Structure
Maintain the following structure for submission based on the workspace:
```text
star_agibot/
├── README.md                         # Project overview
├── LICENSE                           # MIT License (original project code)
├── .gitignore                        # Sensitive information / build‑artifact exclusion rules
│
├── models/                           # Unified storage for vision models (recommended)
├── vits‑zh‑hf‑fanchen‑C/             # Local TTS model / assets (if used)
│
├── agibot_arm64_wheels/              # ARM64 offline Python wheels for core dependencies
├── agibot_yolo_wheels/               # ARM64 YOLO / ONNX offline wheels
├── runtime/                           # Runtime assets
├── scripts/                           # Environment activation and auxiliary runtime scripts
├── final_scripts/                     # Final auxiliary scripts for competition field
├── maps/                              # Map / on‑site auxiliary files (as needed)
├── audio_channel_test/                # Microphone channel test utilities
│
├── check_external_mic.py              # External‑microphone validation
├── send_listen_true.py                # First‑round microphone enable helper script
├── send_pose.py                       # Pose debugging utility
├── test_gray_distribution.py          # Vision grayscale debugging tool
│
└── src/
    ├── voice_asr/                     # 🎙️ Audio capture, VAD, offline ASR
    ├── interaction_agent/             # 🧠 Closed‑set intent + state machine
    ├── x2_execution_pkg/              # 🎛️ Coordinator / Vision / Voice
    ├── race_task/                     # 🧭 Localization bridge, dual navigation, velocity bridge
    ├── grasping/                      # 🦾 Grasping watchdog, executor server, trigger
    ├── x2_ik_sdk/                     # 🧮 IK SDK / offline dependencies
    ├── map_tf_distribution/           # 🗺️ Map / TF support package
    ├── py_examples/                   # 🧪 AimDK examples and reset utilities
    └── ruckig/                        # Third‑party trajectory‑planning dependency
```

## 1.1 Core Package Responsibilities
| Package | Main Responsibilities |
|---|---|
| `voice_asr` | Internal / external microphone access, VAD, SenseVoice / offline ASR; publish recognized text |
| `interaction_agent` | State‑constrained closed‑set semantic recognition; convert raw text into standardized JSON intents |
| `x2_execution_pkg` | Global coordination, vision, offline voice playback, facial expressions / motions, task timing |
| `race_task` | Official‑map localization bridge, Task 1 navigation, service navigation, chassis velocity bridge |
| `grasping` | Gripper watchdog, grasp executor, automatic grasp trigger |
| `x2_ik_sdk` | Inverse‑kinematics SDK and offline dependencies |
| `map_tf_distribution` | Localization / TF / map auxiliary utilities |
| `py_examples` | Robot mode management, posture reset, official‑interface testing |
| `ruckig` | Third‑party trajectory‑planning dependency (use per upstream license) |

---

# 🧩 II. Core Module Description
## 2.1 `voice_asr`
```text
voice_asr/
├── config/                              # Three ASR runtime‑mode configurations
│   ├── voice_asr_raw_internal.yaml      # Internal mic + Raw audio + Silero VAD
│   ├── voice_asr_raw_external.yaml      # External mic + Raw audio + Silero VAD
│   └── voice_asr_processed_external.yaml # External mic + Processed audio + AimDK VAD
├── resource/                            # ROS 2 ament package resource index
├── test/                                # ROS 2 / Python standard tests
├── voice_asr/                           # Core speech‑recognition source code
│   ├── __init__.py                      # Python package initialization
│   ├── voice_asr_node.py                # ASR main node and listen‑control logic
│   ├── asr_engine.py                    # Local offline ASR inference
│   ├── vad_engine.py                    # Silero VAD voice‑activity detection
│   ├── listening_session.py             # Raw‑audio listening and segmentation management
│   ├── robot_audio_source.py            # AimDK Raw‑audio access
│   ├── external_mic.py                  # External‑microphone switching and validation
│   ├── processed_audio_source.py        # AimDK Processed‑audio access
│   └── processed_listening_session.py   # Processed‑audio listening‑session management
├── LICENSE                              # Package‑level open‑source license
├── README.md                            # voice_asr package documentation
├── package.xml                          # ROS 2 package manifest and dependencies
├── setup.cfg                            # Python ROS 2 installation configuration
└── setup.py                             # Python package and executable entry‑point definition
```

Select **one single entry point for competition; do NOT run multiple ASR instances simultaneously**:
```text
voice_asr_raw_internal
    └── Internal microphone + Raw audio + local Silero VAD
voice_asr_raw_external
    └── External microphone + Raw audio + local Silero VAD
voice_asr_processed_external
    └── External microphone + AimDK‑processed audio + AimDK VAD
```

Unified output topic:
```text
/ai_agent/recognized_text
std_msgs/msg/String
```

> The first two entry points have been validated on physical hardware. The final entry point remains untested and requires further tuning.

## 2.2 `interaction_agent`
```text
interaction_agent/
├── interaction_agent/               # 🧠 Core interaction logic
│   ├── __init__.py
│   ├── models.py                    # State / intent enumerations and standard IntentResult
│   ├── rules.py                     # Closed‑set keywords, slots, expression / motion / requirement rules
│   ├── intent_dispatcher.py         # Whitelisted intent dispatch governed by current state
│   ├── state_machine.py             # Finite state machine for competition interactions
│   └── interaction_node.py          # ROS 2 node: text input / JSON output / state synchronization
├── resource/
│   └── interaction_agent
├── test/                            # ROS 2 / Python standard test directory
├── package.xml
├── setup.cfg
└── setup.py
```

Core data flow:
```text
/ai_agent/recognized_text
        ↓
interaction_node
        ↓
rules + intent_dispatcher + state_machine
        ↓
/ai_agent/input_json
```

Sample standardized JSON intent:
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

Major intent types:
| Phase | `intent_type` |
|---|---|
| Task 1 | `task1_go_interaction_area` |
| Basic Interaction | `task3_time_query` |
| Basic Interaction | `task3_digit_color_query` |
| Basic Interaction | `task3_emoji_control` |
| Basic Interaction | `task3_action_control` |
| Autonomous Service | `task4_wake_service` |
| Autonomous Service | `task4_need_classification` |
| Fallback | `unknown` |

State‑machine core transitions:
```text
TASK1_WAIT_COMMAND
        │
        ▼
TASK1_NAVIGATING
        │ task1 reached + listen_control=True
        ▼
INTERACTION_LISTEN
        │
        ├── Time / Digit‑Color / Emoji / Motion
        │
        └── Wake‑up autonomous service
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
├── config/                   # 🌟 Core configuration directory
│   ├── coordinator_nav.yaml  # Navigation and master‑scheduler parameters
│   └── vision_config.yaml    # Desensitized vision‑model path configuration
├── models/                   # 🧠 Local offline model storage
│   ├── best.onnx             # Fine‑tuned grasping‑recognition model (Plan A)
│   ├── yoloe_task4.onnx      # Open‑prompt grasping‑recognition large model (Plan B)
│   ├── yolo.onnx             # Digit‑recognition ONNX model
│   └── yolo_digits.pt        # Original digit‑recognition PT model
├── resource/
├── test/                     # ROS 2 standard test directory
├── x2_execution_pkg/         # 🚀 Core implementation logic
│   ├── __init__.py
│   ├── coordinator_node.py   # Master‑brain coordinator node
│   ├── vision_node.py        # Vision‑perception node
│   └── voice_node.py         # Voice‑interaction node
├── package.xml
├── setup.cfg
└── setup.py
```

- `coordinator_node`: Subscribes to intent JSON, manages microphone state, selects Task‑1 / service navigation, schedules vision, audio playback and grasping actions.
- `vision_node`: Digit / color recognition, target‑object detection, RGB‑D 3D coordinate solving, TF transformations.
- `voice_node`: Local TTS, audio output, microphone policy after playback completes.

Internal interfaces:
```text
/system/voice_cmd
/system/vision_cmd
/system/vision_result
```

## 2.4 `race_task`
```text
race_task/
├── config/                           # 🌟 Physical‑robot navigation parameters
│   ├── competition_official_voice_nav.yaml
│   └── service_official_map_nav.yaml
├── launch/
│   ├── task1_app_ready.launch.py    # Task 1: TF + velocity bridge + task1 navigation
│   └── dual_navigation.launch.py    # Optional: launch both navigation stacks simultaneously for full competition
├── race_task/                        # 🚀 Navigation and chassis core logic
│   ├── __init__.py
│   ├── tf_pose_bridge.py            # Convert map‑to‑base_link TF into PoseStamped
│   ├── task1_pose_nav.py            # Start area → Interaction Area I
│   ├── service_pose_nav.py          # Interaction area → Work / service zones
│   └── cmd_vel_bridge.py            # Forward /cmd_vel to AimDK chassis‑velocity interface
├── resource/
│   └── race_task
├── test/
├── package.xml
├── setup.cfg
└── setup.py
```

Official execution pipeline:
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

Task 1 and service navigation **must use distinct Goal / Status topics**:
```text
Task1:
  /race_task/task1/goal_pose
  /race_task/task1/nav_status
Service:
  /race_task/service/goal_pose
  /race_task/service/nav_status
```

Both navigation nodes may start concurrently, but only enter active control upon receiving their respective goals. Ensure the two navigators never publish valid motion commands at the same time.

## 2.5 `grasping` + `x2_ik_sdk`
Module hierarchy:
```text
star_agibot/
├── src/
│   ├── grasping/                  # Core grasping package (gripper control, server, trigger)
│   ├── x2_ik_sdk/                 # Inverse‑kinematics (IK) offline dependencies and SDK
│   │   ├── offline_deps/          # Offline installation wheels (numpy, pin, etc.)
│   │   └── src/                   # SDK source code
│   └── py_examples/               # Test scripts and reset examples (set_mc_action)
```

Core components:
```text
omnipicker_hand
        └── Gripper low‑level watchdog / state management
x2_grasp_executor_server
        └── Accept grasp requests, perform IK solving, execute robotic‑arm trajectories
x2_grasp_auto_trigger
        └── Listen to /competition/grasp_target and invoke executor
```

Grasp‑target input topic:
```text
/competition/grasp_target
std_msgs/msg/String
```

Sample payload:
```json
{
  "object_name": "cup",
  "target_point": [0.38, 0.0, 0.20]
}
```

`target_point` is a 3‑D coordinate following execution‑chain conventions. In the current vision pipeline coordinates are defined in the `base_link` frame.

---

# 🛠️ III. Environment Requirements
## 3.1 System
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10
- Agibot X2 / AimDK
- `~/aimdk` correctly installed; `source ~/aimdk/install/setup.bash` works
- Physical hardware: NVIDIA Jetson Orin NX / ARM64 recommended
- RGB‑D camera, microphone, loudspeaker
- Official App map and relocalization capability

All regular ROS terminals source environment variables in this fixed order:
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
```

If `scripts/activate_robot_runtime.sh` inside the repository loads offline ASR runtime libraries, append it after the three lines above:
```bash
source ~/star_agibot/scripts/activate_robot_runtime.sh
```

> Do not hard‑code `RMW_IMPLEMENTATION` blindly across different robot images. Use the system default RMW on competition machines; explicitly set it only after confirming ABI compatibility.

## 3.2 Vision / TTS Python Dependencies
Key validated and pinned versions:
```text
numpy==1.24.3
opencv‑python‑headless==4.8.1.78
torch==2.1.0
torchvision==0.16.0
sherpa‑onnx==1.13.4
```

Online development environment installation:
```bash
pip install numpy==1.24.3 opencv‑python‑headless==4.8.1.78
pip install torch==2.1.0 torchvision==0.16.0
pip install sherpa‑onnx==1.13.4 ultralytics onnx onnxruntime
pip install soundfile PyYAML Shapely pyclipper Pillow six
```

## 3.3 ARM64 Offline Installation
Prebuilt wheels are stored in repository root:
```text
agibot_arm64_wheels/
agibot_yolo_wheels/
```

For offline robot operation:
```bash
cd ~/star_agibot/agibot_arm64_wheels
pip install --no-index --find-links=. --no-deps \
  numpy==1.24.3 \
  opencv‑python‑headless==4.8.1.78
pip install --no-index --find-links=. \
  numpy==1.24.3 \
  opencv‑python‑headless==4.8.1.78 \
  sherpa‑onnx==1.13.4 \
  soundfile PyYAML Shapely onnxruntime pyclipper Pillow six
```

YOLO / ONNX offline setup:
```bash
cd ~/star_agibot/agibot_yolo_wheels
pip install --no-index --find-links=. \
  torch==2.1.0 \
  torchvision==0.16.0 \
  onnx onnxruntime \
  numpy==1.24.3 \
  opencv‑python‑headless==4.8.1.78
# Use actual filenames present in directory
pip install --no-index --no-deps ./ultralytics‑*.whl
```

---

# IV. Independent IK‑Grasping Runtime Environment
Robotic‑arm IK runs inside a dedicated Python venv to avoid dependency conflicts with ROS and vision stacks.
```bash
deactivate 2>/dev/null || true
unset PYTHONPATH
export PYTHONNOUSERSITE=1
rm -rf ~/.venvs/x2‑ik‑runtime
python3 -m venv ~/.venvs/x2‑ik‑runtime
source ~/.venvs/x2‑ik‑runtime/bin/activate
cd ~/star_agibot/src/x2_ik_sdk
python3 -m pip install --no-index --find-links=offline_deps \
  numpy pin setuptools wheel
python3 -m pip install --no-index --find-links=offline_deps \
  x2_ik_sdk
```

> Only terminals running grasping logic should activate `x2‑ik‑runtime`. Do NOT activate this venv for ASR, navigation, Coordinator, Vision, Voice or other standard ROS nodes.

---

# V. Build & Compilation
## 5.1 Build Standard ROS Packages
Open a fresh terminal:
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
unset RMW_IMPLEMENTATION
colcon build \
  --symlink‑install \
  --packages‑select \
  voice_asr \
  interaction_agent \
  race_task \
  x2_execution_pkg \
  py_examples
```

After build completes:
```bash
source ~/star_agibot/install/setup.bash
```

Validation check:
```bash
ros2 pkg prefix voice_asr
ros2 pkg prefix interaction_agent
ros2 pkg prefix race_task
ros2 pkg prefix x2_execution_pkg
```

## 5.2 Build Grasping Package
```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2‑ik‑runtime/bin/activate
export PYTHONNOUSERSITE=1
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
colcon build \
  --symlink‑install \
  --packages‑select grasping
```

After build completes:
```bash
source ~/star_agibot/install/setup.bash
```

## 5.3 Verify Executable Entry Points
```bash
ros2 pkg executables voice_asr
ros2 pkg executables interaction_agent
ros2 pkg executables race_task
ros2 pkg executables x2_execution_pkg
ros2 pkg executables grasping
```

Confirm these core executables exist:
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

# VI. Pre‑Launch Physical‑Robot Checks
## 6.1 AimDK Environment
```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
ros2 pkg prefix aimdk_msgs
```

Output must point to valid AimDK installation on robot hardware.

## 6.2 Official Map and Localization
Validate map availability:
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 topic info /map -v
```

Validate TF tree:
```bash
ros2 run tf2_ros tf2_echo map base_link
```

Validate navigation‑consumed localization topic:
```bash
ros2 topic echo \
  /map_tf_distribution/localization_pose \
  --once
```

> Do NOT send real navigation goals if this topic lacks live streaming data.

## 6.3 Dual‑Navigation Interface Validation
```bash
ros2 topic info /race_task/task1/goal_pose --verbose
ros2 topic info /race_task/task1/nav_status --verbose
ros2 topic info /race_task/service/goal_pose --verbose
ros2 topic info /race_task/service/nav_status --verbose
```

Expected for Task 1:
```text
/race_task/task1/goal_pose
Publisher: /coordinator_node
Subscriber: /task1_pose_nav
```

## 6.4 Single‑ASR‑Instance Check
```bash
pgrep -af \
'voice_asr_node|voice_asr_raw_internal|voice_asr_raw_external|voice_asr_processed_external'
```

Only one active ASR process is permitted during competition runs.

## 6.5 External‑Microphone Check (Optional)
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
unset RMW_IMPLEMENTATION
python3 ~/star_agibot/check_external_mic.py
```

---

# VII. Full End‑to‑End Startup Sequence
Start low‑level foundational components first; launch Coordinator last.

## 7.1 Terminal A: Gripper Watchdog
```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2‑ik‑runtime/bin/activate
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"
ros2 run grasping omnipicker_hand --publish close right
```

## 7.2 Terminal B: Grasp Executor Server
```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2‑ik‑runtime/bin/activate
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"
ros2 run grasping x2_grasp_executor_server
```

## 7.3 Terminal C: Automatic Grasp Trigger
```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2‑ik‑runtime/bin/activate
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"
ros2 run grasping x2_grasp_auto_trigger
```

## 7.4 Terminal D: Task 1 Navigation Stack
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 launch race_task \
  task1_app_ready.launch.py \
  params_file:=$HOME/star_agibot/src/race_task/config/competition_official_voice_nav.yaml
```

This launch file brings up:
```text
tf_pose_bridge
cmd_vel_bridge
task1_pose_nav
```

Effective parameters inside `competition_official_voice_nav.yaml`:
```yaml
goal_pose_topic: "/race_task/task1/goal_pose"
nav_status_topic: "/race_task/task1/nav_status"
wait_for_goal_from_rviz: true
```

> The legacy parameter name `wait_for_goal_from_rviz: true` actually means “await external goal input”, where the goal publisher is Coordinator.

## 7.5 Terminal E: Service Navigation
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 run race_task service_pose_nav \
  --ros‑args \
  --params‑file \
  ~/star_agibot/src/race_task/config/service_official_map_nav.yaml
```

Parameter file relevant content:
```yaml
service_pose_nav:
  ros__parameters:
    pose_topic: "/map_tf_distribution/localization_pose"
    cmd_topic: "/cmd_vel"
    goal_pose_topic: "/race_task/service/goal_pose"
    nav_status_topic: "/race_task/service/nav_status"
    wait_for_goal_from_rviz: true
```

Control frequency, maximum velocity, XY / Yaw tolerances are calibrated for physical hardware.

## 7.6 Terminal F: Intent‑Processing Node
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 run interaction_agent interaction_node
```

Initial competition state must be:
```text
TASK1_WAIT_COMMAND
```

## 7.7 Terminal G: ASR (Choose Exactly One)
### Recommended Option 1: External Mic Raw Mode
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
unset RMW_IMPLEMENTATION
ros2 run voice_asr \
  voice_asr_raw_external \
  --ros‑args \
  --params‑file \
  ~/star_agibot/src/voice_asr/config/voice_asr_raw_external.yaml
```

### Option 2: External Mic Processed Mode (Unvalidated on Hardware)
```bash
ros2 run voice_asr \
  voice_asr_processed_external \
  --ros‑args \
  --params‑file \
  ~/star_agibot/src/voice_asr/config/voice_asr_processed_external.yaml
```

### Fallback: Internal Microphone
```bash
ros2 run voice_asr \
  voice_asr_raw_internal \
  --ros‑args \
  --params‑file \
  ~/star_agibot/src/voice_asr/config/voice_asr_raw_internal.yaml
```

> Launch only one of the three options.

## 7.8 Terminal H: Voice Execution Node
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 run x2_execution_pkg voice_node
```

## 7.9 Terminal I: Vision Execution Node
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 run x2_execution_pkg vision_node \
  --ros‑args \
  --params‑file \
  ~/star_agibot/src/x2_execution_pkg/config/vision_config.yaml
```

## 7.10 Terminal J: Coordinator (Start Last)
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
ros2 run x2_execution_pkg coordinator_node \
  --ros‑args \
  --params‑file \
  ~/star_agibot/src/x2_execution_pkg/config/coordinator_nav.yaml
```

After Coordinator startup, verify nodes:
```bash
ros2 node info /coordinator_node
ros2 node info /task1_pose_nav
ros2 node info /service_pose_nav
```

## 7.11 Activate First Listening Cycle
Confirm all nodes healthy, localization valid, robot workspace clear and safe:
```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
python3 ~/star_agibot/send_listen_true.py
```

Then issue voice command:
```text
Please go to interaction area one
```

---

# VIII. Competition Main Workflow
## 8.1 Task 1 → Basic Interaction
```text
"Please go to interaction area one"
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
        ├── Turn off microphone
        └── /race_task/task1/goal_pose
                    ↓
             task1_pose_nav
                    ↓
                 /cmd_vel
                    ↓
              Robot moves
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
Basic‑interaction phase begins
```

## 8.2 Autonomous Service Flow
```text
User wake‑up / requirement utterance
        ↓
interaction_agent
        ↓
task4_need_classification
        ↓
coordinator_node
        ├── TTS reply
        ├── Deactivate microphone
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
         IK solving + Robotic‑arm motion + Gripper operation
```

---

# IX. ROS 2 Interface Summary
| Direction | Topic / Service | Type | Purpose |
|---|---|---|---|
| ASR → Agent | `/ai_agent/recognized_text` | `std_msgs/msg/String` | Recognized speech text |
| Agent → Coordinator | `/ai_agent/input_json` | `std_msgs/msg/String` | Standard intent JSON payload |
| Coordinator → ASR/Agent | `/ai_agent/listen_control` | `std_msgs/msg/Bool` | Microphone toggle and state synchronization |
| Coordinator → Voice | `/system/voice_cmd` | `std_msgs/msg/String` | Audio‑playback command JSON |
| Coordinator → Vision | `/system/vision_cmd` | `std_msgs/msg/String` | Vision‑task command JSON |
| Vision → Coordinator | `/system/vision_result` | `std_msgs/msg/String` | Vision‑detection result JSON |
| Coordinator → Task1 Nav | `/race_task/task1/goal_pose` | `geometry_msgs/msg/PoseStamped` | Task 1 navigation target pose |
| Task1 Nav → Coordinator | `/race_task/task1/nav_status` | `std_msgs/msg/String` | Navigation state, `"reached"` on arrival |
| Coordinator → Service Nav | `/race_task/service/goal_pose` | `geometry_msgs/msg/PoseStamped` | Service‑task navigation target pose |
| Service Nav → Coordinator | `/race_task/service/nav_status` | `std_msgs/msg/String` | Navigation state, `"reached"` on arrival |
| TF Bridge → Nav | `/map_tf_distribution/localization_pose` | `geometry_msgs/msg/PoseStamped` | Real‑time robot pose in map frame |
| Nav → Speed Bridge | `/cmd_vel` | `geometry_msgs/msg/Twist` | Navigation output velocity commands |
| Speed Bridge → AimDK | `/aima/mc/locomotion/velocity` | AimDK custom message | Low‑level chassis velocity input |
| Vision → Grasp | `/competition/grasp_target` | `std_msgs/msg/String` | 3‑D target for grasping pipeline |
| Vision Input | `/aima/hal/sensor/rgbd_head_front/rgb_image` | `sensor_msgs/msg/Image` | RGB camera image stream |
| Vision Input | `/aima/hal/sensor/rgbd_head_front/depth_image` | `sensor_msgs/msg/Image` | Depth camera image stream |
| Vision Input | `/aima/hal/sensor/rgbd_head_front/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsic parameters |
| Voice → Audio | `/aima/hal/audio/playback` | `aimdk_msgs/msg/AudioPlayback` | Audio playback interface |
| Coordinator → Face | `PlayEmoji`‑related AimDK Service | AimDK Service | Robot facial‑expression control |
| Coordinator → Motion | `SetMcPresetMotion`‑related AimDK Service | AimDK Service | Preset robot motion execution |

---

# X. Module‑by‑Module Integration Testing
## 10.1 Test Agent Only (No Robot Motion)
Launch only `interaction_agent` and monitor output:
```bash
ros2 topic echo /ai_agent/input_json
```

Simulate ASR input manually:
```bash
ros2 topic pub --once \
  /ai_agent/recognized_text \
  std_msgs/msg/String \
  "{data: 'Please go to interaction area one'}"
```

Expected output contains:
```text
task1_go_interaction_area
```

## 10.2 Simulate Task‑1 Arrival Event
```bash
ros2 topic pub --once \
  /ai_agent/listen_control \
  std_msgs/msg/Bool \
  "{data: true}"
```

State should transition from:
```text
TASK1_NAVIGATING
```
to:
```text
INTERACTION_LISTEN
```

## 10.3 Grasp‑Chain End‑to‑End Smoke Test
**Ensure robotic‑arm workspace is clear and no personnel are nearby before running:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once \
  /competition/grasp_target \
  std_msgs/msg/String \
  "data: '{\"object_name\":\"test_cup\",\"target_point\":[0.38,0.0,0.20]}'"
```

## 10.4 Robotic‑Arm Reset Command
```bash
deactivate 2>/dev/null || true
source ~/.venvs/x2‑ik‑runtime/bin/activate
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/star_agibot/install/setup.bash
export PYTHONPATH="$HOME/star_agibot/src/x2_ik_sdk/src:$PYTHONPATH"
cd ~/star_agibot/src/py_examples
python3 -m py_examples.set_mc_action SD
```

---

# XI. Safety Conventions
1. **Starting navigation nodes must not trigger immediate robot movement.** Navigators must wait for externally‑supplied goal poses.
2. Before issuing real navigation goals, verify `/map_tf_distribution/localization_pose` outputs valid live data.
3. Task‑1 and Service navigators shall never simultaneously stay in active control states.
4. Upon navigation completion, exceptions, timeouts or node shutdown, publish zero velocity to chassis.
5. Only one ASR instance may run concurrently to prevent duplicate speech recognition and duplicate intent JSON outputs.
6. Mute microphones during audio playback and chassis motion to avoid self‑excitation and false triggers from motor noise.
7. Clear the workspace before robotic‑arm / gripper testing; prefer low‑speed operation and safe home postures.
8. Do not send unvalidated raw 3‑D coordinates to `/competition/grasp_target` during debugging.
9. Never forward raw cloud‑model outputs directly as motion commands; execute only locally‑whitelisted intents.
10. After flashing new system images on competition hardware, validate AimDK, RMW and Fast‑CDR message ABI compatibility before launching upper‑layer application nodes.

---

# XII. Common Troubleshooting
### 12.1 `task1_pose_nav` does not receive navigation goals
```bash
ros2 topic info /race_task/task1/goal_pose --verbose
```

Output should show both:
```text
/coordinator_node
/task1_pose_nav
```

Check YAML config files for legacy topic names:
```text
/race_task/nav_goal
/goal_pose
```

The project uses standardized topic:
```text
/race_task/task1/goal_pose
```

### 12.2 Localization topic has no data
```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /map_tf_distribution/localization_pose --once
```

Fix official App relocalization and TF tree first; then test physical‑robot navigation.

### 12.3 Single utterance processed multiple times
Check for multiple ASR instances running:
```bash
pgrep -af \
'voice_asr_node|voice_asr_raw_internal|voice_asr_raw_external|voice_asr_processed_external'
```

Keep exactly one ASR process active.

### 12.4 Python / IK dependency pollution
Grasp‑related terminals must use:
```bash
~/.venvs/x2‑ik‑runtime
```

Do NOT activate this virtual environment for regular ROS nodes.

### 12.5 Model‑file path not found
Avoid hard‑coding absolute file paths inside source code. Inspect:
```bash
~/star_agibot/src/x2_execution_pkg/config/vision_config.yaml
```

Point model entries to actual local directories such as `~/star_agibot/models/`.

---

# XIII. License and Third‑Party Components
Original source code within this repository is released under the [MIT License](LICENSE).

The following assets originate from third parties and retain their original licenses independent of the repository‑level MIT statement:
- Agibot AimDK / `aimdk_msgs`;
- `x2_ik_sdk` and its binary / offline dependencies;
- `ruckig`;
- ONNX / PyTorch / Ultralytics / Sherpa‑ONNX;
- Pre‑trained models, fonts, speech models and datasets;
- Robot‑vendor‑provided maps, SDK artifacts and resource files.

Review upstream license terms and competition‑organizer authorization requirements before redistribution or public release.