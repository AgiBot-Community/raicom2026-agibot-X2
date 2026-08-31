#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import sys
from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig

def main():
    parser = argparse.ArgumentParser(description="Offline X2 arm IK demo")
    parser.add_argument('--side', type=str, required=True, choices=['left', 'right'], help="Select arm: left or right")
    parser.add_argument('--target-offset', type=float, nargs=3, help="Target offset X Y Z")
    parser.add_argument('--keep-current-rpy', action='store_true', help="Keep current rpy")
    parser.add_argument('--print-json', action='store_true', help="Print result in JSON")
    args = parser.parse_args()

    try:
        solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
        side_enum = ArmSide.RIGHT if args.side == 'right' else ArmSide.LEFT
        
        seed_pos = solver.ready_arm_pos()
        current_xyz = solver.fk_xyz(side_enum, seed_pos)
        
        target_xyz = current_xyz
        if args.target_offset:
            target_xyz = [
                current_xyz[0] + args.target_offset[0],
                current_xyz[1] + args.target_offset[1],
                current_xyz[2] + args.target_offset[2]
            ]
                
        result = solver.solve_position(
            side=side_enum,
            target_xyz=target_xyz,
            current_arm_pos=seed_pos
        )
        
        if args.print_json:
            out = {
                "success": result.success,
                "message": result.message,
                "error_norm": result.error_norm,
                "arm_pos": result.arm_pos
            }
            print(json.dumps(out, indent=2))
        else:
            print(f"success: {result.success}")
            print(f"message: {result.message}")
            print(f"error_norm: {result.error_norm:.6f}")
            
    except Exception as e:
        if args.print_json:
            print(json.dumps({"success": False, "message": str(e)}))
        else:
            print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()