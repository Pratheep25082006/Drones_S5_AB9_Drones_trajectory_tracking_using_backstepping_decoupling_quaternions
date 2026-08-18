"""
Target Setter Utility Script for Quadrotor Simulation
-----------------------------------------------------
Usage:
  1) Fly to specific coordinates:
     python set_target.py 5 5 10

  2) Resume automatic 3D trajectory (Helical + Figure-8):
     python set_target.py auto
"""

import sys
import os

target_file = r"d:\Drones\target.txt"

if len(sys.argv) >= 2 and sys.argv[1].lower() in ["auto", "reset"]:
    if os.path.exists(target_file):
        os.remove(target_file)
    print("[SUCCESS] Resumed Automatic 3D Demo Trajectory (Helical Spiral & Figure-8)!")

elif len(sys.argv) == 4:
    try:
        x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        with open(target_file, "w") as f:
            f.write(f"{x} {y} {z}")
        print(f"[SUCCESS] Target set to ({x:.1f}m, {y:.1f}m, {z:.1f}m)!")
        print(f"[INFO] Quadrotor in Gazebo navigating to target setpoint...")
    except ValueError:
        print("[ERROR] Please enter numeric values for X Y Z (e.g., python set_target.py 5 5 10)")

else:
    print("=========================================================")
    print("           QUADROTOR TARGET SETTER USAGE                 ")
    print("=========================================================")
    print("  To fly to custom coordinates (X, Y, Z):")
    print("    python set_target.py 5 5 10")
    print("    python set_target.py -2 3 5")
    print()
    print("  To resume automatic trajectory mode:")
    print("    python set_target.py auto")
    print("=========================================================")
