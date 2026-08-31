# voice_asr 三版本比赛包

本包保留三条可独立启动的离线语音识别链路，业务接口完全相同：

- 订阅：`/ai_agent/listen_control` (`std_msgs/msg/Bool`)
- 发布：`/ai_agent/recognized_text` (`std_msgs/msg/String`)
- ASR：本地 SenseVoice INT8

## 版本1：内置麦 + 原始音频 + 本地 Silero VAD

入口：`voice_asr_raw_internal`

链路：

`mic_source=0` -> `/aima/hal/audio/capture` -> 6总声道(4 mic + 2 ref) -> mic ch2 -> gain 14.0 -> Silero VAD -> SenseVoice

对应配置：`config/voice_asr_raw_internal.yaml`

这是最初已经跑通的内置麦原始音频方案。

## 版本2：外置麦 + 原始音频 + 本地 Silero VAD

入口：`voice_asr_raw_external`

链路：

`mic_source=1` -> `/aima/hal/audio/capture` -> 2总声道(1 mic + 1 ref) -> mic ch0 -> gain 1.0 -> Silero VAD -> SenseVoice

对应配置：`config/voice_asr_raw_external.yaml`

外置麦实测：静音约 -55.25 dBFS，正常说话约 -29.30 dBFS，因此不再使用 14 倍增益。

## 版本3：外置麦 + 处理后音频 + AimDK VAD

入口：`voice_asr_processed_external`

链路：

`mic_source=1` -> `/agent/process_audio_output` -> AimDK `BEGIN/PROCESSING/END` -> 直接拼接完整 PCM -> SenseVoice

此模式**不运行 Silero VAD**。

对应配置：`config/voice_asr_processed_external.yaml`

实机已验证：

- `audio_vad_state=1`：BEGIN
- `audio_vad_state=2`：PROCESSING
- `audio_vad_state=3`：END
- `audio_data` 为 `uint8[]` / Python `array.array`，包长不固定。

## 向后兼容入口

`voice_asr_node` 默认等价于 `voice_asr_raw_internal`，即恢复最初的内置麦原始音频版本。

## 编译

```bash
cd ~/star_agibot
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
unset RMW_IMPLEMENTATION
colcon build --symlink-install --packages-select voice_asr
```

## 启动

内置麦原始：

```bash
ros2 run voice_asr voice_asr_raw_internal --ros-args --params-file ~/star_agibot/src/voice_asr/config/voice_asr_raw_internal.yaml
```

外置麦原始：

```bash
ros2 run voice_asr voice_asr_raw_external --ros-args --params-file ~/star_agibot/src/voice_asr/config/voice_asr_raw_external.yaml
```

外置麦处理后：

```bash
ros2 run voice_asr voice_asr_processed_external --ros-args --params-file ~/star_agibot/src/voice_asr/config/voice_asr_processed_external.yaml
```

三个版本不要同时运行，否则同一句话会被重复发布到 `/ai_agent/recognized_text`。
