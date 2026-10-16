"""
Target & Trajectory Command Utility for Quadrotor Simulation
-----------------------------------------------------------
Commands the quadrotor flight controller in real time.

Usage Examples:
  1) Fly to specific 3D coordinates:
     python set_target.py 5 5 10
     python set_target.py -3 4 6

  2) Select Trajectory Flight Modes:
     python set_target.py auto        (Mission: takeoff -> hover -> paper helix (Fig. 4) -> figure-8, looping)
     python set_target.py spiral      (Paper vertical helix, Fig. 4)
     python set_target.py figure8     (3D lemniscate figure-8)
     python set_target.py hover       (Hold (0, 0, 1 m))
     python set_target.py land        (Return to the helipad and descend)
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quadrotor_controller.config import banner, project_path  # noqa: E402

target_file = project_path("target_file")

if len(sys.argv) < 2:
    print("=================================================================")
    print("         QUADROTOR TRAJECTORY & TARGET COMMAND UTILITY           ")
    print("=================================================================")
    print("  [1] Custom Coordinates:")
    print("      python set_target.py <X> <Y> <Z> [heading_deg]")
    print("      Example: python set_target.py 5 5 8")
    print()
    print("  [2] Automated Trajectory Flight Modes:")
    print("      python set_target.py auto        - Mission: takeoff -> helix -> figure-8 (looping)")
    print("      python set_target.py spiral      - Paper vertical helix (Fig. 4)")
    print("      python set_target.py figure8     - 3D figure-8")
    print("      python set_target.py hover       - Hold (0, 0, 1 m)")
    print("      python set_target.py land        - Return to helipad and land")
    print("=================================================================")
    sys.exit(0)

cmd = sys.argv[1].lower()

# Check for trajectory modes
TRAJECTORY_MODES = {
    "auto": "Mission: takeoff -> hover -> paper helix (Fig. 4) -> figure-8, looping",
    "mission": "Mission: takeoff -> hover -> paper helix (Fig. 4) -> figure-8, looping",
    "spiral": "Paper vertical helix (De Monte & Lohmann 2013, Fig. 4)",
    "figure8": "3D lemniscate figure-8",
    "land": "Return to the helipad and descend",
    "hover": "Position hold at (0, 0, 1 m)",
}

if cmd in TRAJECTORY_MODES:
    mode_name = "auto" if cmd in ["auto", "mission"] else cmd
    with open(target_file, "w") as f:
        f.write(f"MODE {mode_name}")
    print(banner())
    print(f" [FLIGHT MODE ACTIVATED] {mode_name.upper()}")
    print(f" Description: {TRAJECTORY_MODES[cmd]}")
    print(" Target beacon and flight controller updated in real time!")
    print(banner())

elif len(sys.argv) >= 4:
    try:
        x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        heading = float(sys.argv[4]) if len(sys.argv) >= 5 else 0.0
        with open(target_file, "w") as f:
            f.write(f"{x} {y} {z} {heading}")
        print(banner())
        print(f" [CUSTOM TARGET SET] Position: ({x:+.2f}m, {y:+.2f}m, {z:+.2f}m)")
        print(f" Heading: {heading:+.1f} deg")
        print(" Dynamic target beacon moved! Quadrotor navigating smoothly...")
        print(banner())
    except ValueError:
        print("[ERROR] Coordinates must be numbers (e.g., python set_target.py 5 5 10)")
        sys.exit(1)
else:
    print(f"[ERROR] Unrecognized command '{sys.argv[1]}'. Run 'python set_target.py' for help.")
    sys.exit(1)

