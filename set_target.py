"""
Target & Trajectory Command Utility for Quadrotor Simulation
-----------------------------------------------------------
Commands the quadrotor flight controller in real time.

Usage Examples:
  1) Fly to specific 3D coordinates:
     python set_target.py 5 5 10
     python set_target.py -3 4 6

  2) Select Trajectory Flight Modes:
     python set_target.py mission     (Full test mission: Takeoff -> Slalom Gates -> Spiral -> Figure-8 -> Land)
     python set_target.py racetrack   (High-speed racing through arena slalom gates)
     python set_target.py spiral      (Dynamic 3D helical spiral climb & dive)
     python set_target.py figure8     (3D Lemniscate Figure-8 aerobatics)
     python set_target.py land        (Precision RTL & landing on central Helipad)
     python set_target.py auto        (Default automated flight mission)
"""

import sys
import os

target_file = r"d:\Drones\target.txt"

if len(sys.argv) < 2:
    print("=================================================================")
    print("         QUADROTOR TRAJECTORY & TARGET COMMAND UTILITY           ")
    print("=================================================================")
    print("  [1] Custom Coordinates:")
    print("      python set_target.py <X> <Y> <Z> [heading_deg]")
    print("      Example: python set_target.py 5 5 8")
    print()
    print("  [2] Automated Trajectory Flight Modes:")
    print("      python set_target.py mission     - 5-Phase Full Test Flight")
    print("      python set_target.py racetrack   - Slalom Gates Arena Circuit")
    print("      python set_target.py spiral      - 3D Helical Spiral Climb")
    print("      python set_target.py figure8     - 3D Figure-8 Aerobatics")
    print("      python set_target.py land        - Precision Helipad Landing")
    print("      python set_target.py auto        - Default Auto Mission")
    print("=================================================================")
    sys.exit(0)

cmd = sys.argv[1].lower()

# Check for trajectory modes
TRAJECTORY_MODES = {
    "auto": "Automated Multi-Phase Mission (Takeoff -> Gates -> Spiral -> Figure-8 -> Land)",
    "mission": "Automated Multi-Phase Mission (Takeoff -> Gates -> Spiral -> Figure-8 -> Land)",
    "racetrack": "Arena Gate Racetrack Navigation (Gates East -> North -> West -> South)",
    "gates": "Arena Gate Racetrack Navigation (Gates East -> North -> West -> South)",
    "spiral": "3D Helical Spiral Climb and Descent",
    "figure8": "3D Lemniscate Figure-8 Aerobatics",
    "land": "Precision Return-to-Launch and Helipad Touchdown",
    "hover": "Stationary Position Hold Hover",
}

if cmd in TRAJECTORY_MODES:
    mode_name = "auto" if cmd in ["auto", "mission"] else cmd
    with open(target_file, "w") as f:
        f.write(f"MODE {mode_name}")
    print("=" * 65)
    print(f" [FLIGHT MODE ACTIVATED] {mode_name.upper()}")
    print(f" Description: {TRAJECTORY_MODES[cmd]}")
    print(" Target beacon and flight controller updated in real time!")
    print("=" * 65)

elif len(sys.argv) >= 4:
    try:
        x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        heading = float(sys.argv[4]) if len(sys.argv) >= 5 else 0.0
        with open(target_file, "w") as f:
            f.write(f"{x} {y} {z} {heading}")
        print("=" * 65)
        print(f" [CUSTOM TARGET SET] Position: ({x:+.2f}m, {y:+.2f}m, {z:+.2f}m)")
        print(f" Heading: {heading:+.1f} deg")
        print(" Dynamic target beacon moved! Quadrotor navigating smoothly...")
        print("=" * 65)
    except ValueError:
        print("[ERROR] Coordinates must be numbers (e.g., python set_target.py 5 5 10)")
        sys.exit(1)
else:
    print(f"[ERROR] Unrecognized command '{sys.argv[1]}'. Run 'python set_target.py' for help.")
    sys.exit(1)

