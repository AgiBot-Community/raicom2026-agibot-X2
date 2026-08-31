from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig


def main():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    arm_pos = solver.ready_arm_pos()

    side = ArmSide.RIGHT
    current_xyz = solver.fk_xyz(side, arm_pos)
    target_xyz = [current_xyz[0] + 0.01, current_xyz[1], current_xyz[2] + 0.01]

    result = solver.solve_position(side, target_xyz, arm_pos)
    print("success:", result.success)
    print("error_norm:", result.error_norm)
    print("arm_pos:", result.arm_pos)


if __name__ == "__main__":
    main()
