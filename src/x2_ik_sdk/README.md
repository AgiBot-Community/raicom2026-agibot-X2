# x2_ik_sdk

基于 Pinocchio 的 X2 双臂离线运动学 Python SDK。

## 已实现功能

- 左臂和右臂末端位置 IK；
- 末端位置 `xyz` 和姿态 `rpy` 的正运动学；
- `arm_pos[14]` 与 Pinocchio configuration 的相互转换；
- ready pose、关节限位和迭代关节限位保护；
- `current_arm_pos`、`current_head_pos` 和 `q_seed` 初值；
- 内置 X2 omnipicker 纯运动学 URDF。

## 待补充功能

当前仅实现末端位置 IK，需要补充同时约束末端位置 `xyz` 与末端姿态 `rpy` 的
末端姿态 IK。

## 安装

```bash
python -m pip install --no-build-isolation -e .
```

依赖：

```text
numpy
pin
```

## Python API

```python
from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig

solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
arm_pos = solver.ready_arm_pos()

current_xyz = solver.fk_xyz(ArmSide.RIGHT, arm_pos)
target_xyz = [current_xyz[0] + 0.01, current_xyz[1], current_xyz[2] + 0.01]

result = solver.solve_position(
    side=ArmSide.RIGHT,
    target_xyz=target_xyz,
    current_arm_pos=arm_pos,
)

print(result.success)
print(result.error_norm)
print(result.arm_pos)
```

也可以运行仓库内示例：

```bash
python examples/offline_demo.py
```

## 模型

默认 URDF：

```text
src/x2_ik_sdk/resources/x2_ultra_plus_omnipicker_omnipicker.urdf
```

`arm_pos[14]` 的顺序由 `x2_ik_sdk.config.ARM_POS_ORDER` 定义，依次为左臂
7 个关节和右臂 7 个关节。
