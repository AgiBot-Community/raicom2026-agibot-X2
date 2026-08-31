from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig


def test_right_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.RIGHT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.RIGHT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14


def test_left_arm_offset_ik():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    seed = solver.ready_arm_pos()
    current = solver.fk_xyz(ArmSide.LEFT, seed)
    target = [current[0] + 0.01, current[1], current[2] + 0.01]
    result = solver.solve_position(ArmSide.LEFT, target, seed)
    assert result.success
    assert result.error_norm < 2e-4
    assert len(result.arm_pos) == 14
