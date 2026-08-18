# 🚁 Quadrotor Backstepping Controller & Gazebo 11 Simulation

A real-time **quadrotor flight simulation** implementing a **Backstepping Quaternion Controller** for trajectory tracking. The drone executes autonomous 3D flight trajectories (Helical Spiral → Figure-8 Loop) inside **Native Gazebo 11** running via **WSL2 Ubuntu** on Windows, with a live **WebSocket web visualizer** for telemetry monitoring.

> Based on the research paper:
> *"Trajectory Tracking Control for a Quadrotor Helicopter Based on Backstepping Using a Decoupling Quaternion Parametrization"* — De Monte & Lohmann (2013)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Architecture](#-architecture)
- [Project Structure](#-project-structure)
- [Prerequisites & Installation](#-prerequisites--installation)
- [Quick Start](#-quick-start)
- [Running the Simulation](#-running-the-simulation)
- [Web Visualizer](#-web-visualizer)
- [Flight Modes](#-flight-modes)
- [Setting a Custom Target](#-setting-a-custom-target)
- [Controller Design](#-controller-design)
- [API Reference](#-api-reference)
- [Running Tests](#-running-tests)
- [Troubleshooting](#-troubleshooting)

---

## 🔍 Overview

This project implements a full-stack quadrotor simulation pipeline:

| Layer | Technology |
|---|---|
| **Flight Controller** | Python — Backstepping Quaternion Algorithm |
| **3D Simulator** | Gazebo 11 (native inside WSL2 Ubuntu) |
| **X11 Display** | VcXsrv X-Server (Windows Host) |
| **Telemetry Bridge** | Python WebSocket server + UDP socket |
| **Web Visualizer** | HTML/JS real-time 3D telemetry dashboard |
| **MAVLink Bridge** | pymavlink (optional — ArduPilot SITL support) |

The controller runs at **50 Hz** (20 ms loop), streams drone pose to Gazebo via **UDP on port 9090**, and broadcasts live telemetry to browser clients via **WebSocket on port 8765**.

---

## 🏗 Architecture

```
Windows Host
│
├── run_gazebo_simulation.py   ← Main simulation entry point (50 Hz control loop)
│     │
│     ├── BacksteppingController   ← Computes thrust + torque commands
│     ├── SimulationRunner         ← Integrates kinematics, streams UDP pose
│     └── file_listener_loop       ← Monitors target.txt for target updates
│
├── run_gazebo_bridge.py       ← WebSocket bridge + HTTP file server
│     │
│     ├── WebSocket Server :8765   ← Pushes JSON telemetry to browser
│     └── HTTP Server :8080        ← Serves gazebo_visualizer.html
│
└── set_target.py              ← CLI utility to set/reset drone target
        │
        └── target.txt              ← Shared IPC file (X Y Z setpoint)

WSL2 Ubuntu
│
├── gzserver                   ← Gazebo physics server
├── gzclient                   ← Gazebo 3D GUI (displayed via VcXsrv)
└── UDP Listener :9090         ← Receives pose from Windows → updates model
```

---

## 📁 Project Structure

```
d:\Drones\
│
├── quadrotor_controller/          # Core Python controller package
│   ├── __init__.py                # Public API exports
│   ├── controller.py              # BacksteppingController, quaternion math
│   └── sitl_bridge.py             # SITLBridge — MAVLink & simulator bridge
│
├── models/
│   └── quadrotor.world            # Gazebo 11 SDF world file
│
├── tests/
│   ├── test_controller.py         # Unit tests for BacksteppingController
│   └── test_sitl_bridge.py        # Unit tests for SITLBridge
│
├── run_gazebo_simulation.py       # ⭐ Main simulation runner (Windows)
├── run_gazebo_bridge.py           # WebSocket telemetry bridge server
├── gazebo_visualizer.html         # Real-time web dashboard (3D telemetry UI)
├── set_target.py                  # CLI target setter utility
├── launch_gazebo_wsl.py           # Launches Gazebo inside WSL2 from Python
├── run_gazebo.sh                  # Bash launcher (run inside WSL Ubuntu)
│
├── START_GAZEBO_GUI.bat           # 1-click Windows launcher (Gazebo GUI)
├── launch_web_visualizer.bat      # 1-click launcher for web visualizer
│
├── target.txt                     # Runtime IPC file — stores X Y Z setpoint
├── GAZEBO_INSTALLATION_GUIDE.md   # Full Gazebo 11 + WSL2 setup guide
└── Trajectory_tracking_control... # Reference research paper (PDF)
```

---

## 🛠 Prerequisites & Installation

### System Requirements

- **OS:** Windows 10 / 11 (with WSL2 enabled)
- **WSL2:** Ubuntu 22.04 LTS or Ubuntu 20.04 LTS
- **Python:** 3.9+ (Windows side)
- **X11 Server:** VcXsrv (for Gazebo GUI)
- **Gazebo:** Version 11.x (installed via Pixi inside WSL2)

### 1. Install VcXsrv (X11 Display Server)

```powershell
winget install marha.VcXsrv
```

Launch **XLaunch**, choose *Multiple Windows*, display `0`, and enable **"Disable access control"**.

### 2. Install WSL2 & Ubuntu

```powershell
wsl --install -d Ubuntu
```

### 3. Install Gazebo 11 via Pixi (inside WSL Ubuntu)

```bash
# Install Pixi package manager
curl -fsSL https://pixi.sh/install.sh | bash
source ~/.bashrc

# Install Gazebo 11
pixi global install gazebo
```

### 4. Install OpenGL & X11 dependencies (WSL Ubuntu)

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential mesa-utils libgl1-mesa-dri \
  libgl1-mesa-glx libglx-mesa0 \
  libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4
```

### 5. Install Python dependencies (Windows)

```powershell
cd d:\Drones
python -m venv .venv
.venv\Scripts\activate
pip install pymavlink websockets
```

> **Note:** `pymavlink` is optional. The simulation runs in pure-Python mode without it. It is only needed for real ArduPilot SITL / hardware MAVLink connections.

For full setup instructions, see [`GAZEBO_INSTALLATION_GUIDE.md`](./GAZEBO_INSTALLATION_GUIDE.md).

---

## ⚡ Quick Start

### Option A — 1-Click GUI Launch (Windows)

Double-click **`START_GAZEBO_GUI.bat`** — this automatically:
1. Starts VcXsrv X-Server (if not running)
2. Launches Gazebo 11 inside WSL2 with the quadrotor world
3. Starts the Backstepping controller

### Option B — Manual Launch

**Terminal 1 — Start Simulation:**
```powershell
cd d:\Drones
python run_gazebo_simulation.py
```

**Terminal 2 — Start Web Visualizer (optional):**
```powershell
cd d:\Drones
python run_gazebo_bridge.py
```
Then open `http://localhost:8080` in your browser.

---

## 🚀 Running the Simulation

### Basic Run (automatic trajectory mode)

```powershell
python run_gazebo_simulation.py
```

The drone will automatically execute the **3D demo trajectory loop**:
1. **Takeoff** — Hover at (0, 0, 2m)
2. **Helical Spiral** — Circular orbit with altitude oscillation
3. **3D Figure-8 Loop** — Lemniscate in 3D space
4. **Repeat** (40-second cycle)

### Run with a Custom Target

Pass `X Y Z` coordinates directly as command-line arguments:

```powershell
python run_gazebo_simulation.py 5 5 10
```

The drone will immediately navigate to position (5m, 5m, 10m altitude).

### Launch via WSL Bash Script (inside WSL Ubuntu)

```bash
sudo bash /mnt/d/Drones/run_gazebo.sh
```

---

## 🌐 Web Visualizer

Launch the WebSocket telemetry bridge:

```powershell
python run_gazebo_bridge.py
```

Then open your browser at:

```
http://localhost:8080
```

The dashboard shows live:
- 3D drone position (X, Y, Z)
- Reference trajectory vs. actual position
- Thrust output (N)
- Torque commands (roll, pitch, yaw)
- Position tracking error
- Flight mode (Helix / Setpoint)
- Connection status

> You can also double-click **`launch_web_visualizer.bat`** to start the bridge and open the browser automatically.

---

## ✈️ Flight Modes

The simulation supports two flight modes:

### 1. Trajectory Mode (Auto)

The drone autonomously executes a repeating 40-second cycle:

| Phase | Duration | Description |
|---|---|---|
| **Hover** | 0–4 s | Stabilize at (0, 0, 2m) |
| **Helical Spiral** | 4–22 s | Circular orbit (r=2m) with altitude oscillation |
| **3D Figure-8** | 22–40 s | Lemniscate of Bernoulli in 3D |

### 2. Target Setpoint Mode

The drone holds position at a custom (X, Y, Z) coordinate. Activated by `set_target.py` or passing CLI arguments.

---

## 🎯 Setting a Custom Target

Use the **`set_target.py`** utility while the simulation is running:

```powershell
# Fly to X=5m, Y=5m, Z=10m altitude
python set_target.py 5 5 10

# Fly to negative coordinates
python set_target.py -2 3 5

# Resume automatic 3D trajectory
python set_target.py auto
```

The utility writes to `target.txt`, which the simulation monitors in real time (polling every 500 ms). No restart required.

---

## 🧮 Controller Design

### Algorithm: Backstepping with Decoupled Quaternion Parametrization

The controller is based on the **Backstepping** nonlinear control method, using a **quaternion attitude representation** to avoid gimbal lock. The attitude is decomposed into:

- **q_xy** — Tilt quaternion (roll + pitch)
- **q_z** — Heading quaternion (yaw)

This decoupled parametrization (De Monte & Lohmann, 2013) allows independent tilt and yaw control.

### Control Law

At each 20 ms step, the controller computes:

```
thrust, torques = controller.compute_command(state, reference, dt)
```

**Position error → Desired acceleration:**

$$\ddot{x}_{des} = \ddot{x}_{ref} + k_{pos}(x_{ref} - x) + k_{vel}(\dot{x}_{ref} - \dot{x})$$

**Desired acceleration → Thrust vector:**

$$\vec{T} = m \cdot g \hat{z} + m \cdot \ddot{x}_{des}$$

**Thrust direction → Attitude error → Roll/Pitch torques:**

$$\tau_{rp} = k_{tilt} \cdot (\hat{z}_{actual} \times \hat{z}_{desired}) - k_{rate} \cdot \omega_{rp}$$

**Heading error → Yaw torque:**

$$\tau_{yaw} = k_{yaw} \cdot \psi_{error} - k_{rate} \cdot \omega_{yaw}$$

### Controller Gains (defaults)

| Parameter | Value | Description |
|---|---|---|
| `mass` | 1.5 kg | Vehicle mass |
| `gravity` | 9.81 m/s² | Gravitational acceleration |
| `k_position` | 2.0 | Position error gain |
| `k_velocity` | 1.5 | Velocity error gain |
| `k_tilt` | 3.5 | Tilt (roll/pitch) attitude gain |
| `k_yaw` | 2.5 | Yaw error gain |
| `k_rate` | 0.8 | Angular rate damping gain |

### Safety Limits

| Limit | Value |
|---|---|
| Max horizontal acceleration | 5.0 m/s² |
| Max thrust | 2.5 × hover thrust (~36.8 N) |
| Minimum altitude | 0.2 m |

---

## 📦 API Reference

### `quadrotor_controller.BacksteppingController`

```python
from quadrotor_controller import BacksteppingController, ControllerState, ControllerReference

controller = BacksteppingController(mass=1.5, gravity=9.81)

state = ControllerState(
    position=(x, y, z),
    velocity=(vx, vy, vz),
    quaternion=(w, x, y, z),   # unit quaternion
    angular_rate=(wx, wy, wz),
)

reference = ControllerReference(
    position=(rx, ry, rz),
    velocity=(rvx, rvy, rvz),
    acceleration=(rax, ray, raz),
    heading=0.0,               # yaw in radians
)

thrust, torques = controller.compute_command(state, reference, dt=0.02)
# thrust  → scalar (Newtons)
# torques → (roll_torque, pitch_torque, yaw_torque)
```

### `quadrotor_controller.SITLBridge`

```python
from quadrotor_controller import SITLBridge

bridge = SITLBridge(connection_string="udp:127.0.0.1:14550")
bridge.connect(timeout=10.0)
bridge.set_mode("GUIDED")
bridge.arm_vehicle(arm=True)

# Send attitude target
target = bridge.build_attitude_target(thrust=14.7, roll=0.0, pitch=0.05, yaw=0.0, yaw_rate=0.0)
bridge.send_attitude_target(target)

bridge.disconnect()
```

### Quaternion Utilities

```python
from quadrotor_controller import (
    quaternion_from_euler,    # (roll, pitch, yaw) → (w, x, y, z)
    normalize_quaternion,     # q → unit quaternion
    decompose_attitude,       # q → (q_xy, q_z) tilt/heading split
)
```

---

## 🧪 Running Tests

The project includes unit tests for the controller and SITL bridge:

```powershell
cd d:\Drones
python -m pytest tests/ -v
```

**Test coverage includes:**

| Test | Description |
|---|---|
| `test_normalize_quaternion` | Verifies unit quaternion normalization |
| `test_decompose_attitude_recombines_to_original_quaternion` | Validates tilt/heading decomposition and recombination |
| `test_controller_returns_positive_thrust_and_three_torques` | Ensures controller output shape and sign correctness |

---

## 🔧 Troubleshooting

### Gazebo window doesn't open / stays black

- Make sure **VcXsrv** is running with *"Disable access control"* checked.
- Verify the DISPLAY variable in WSL: `echo $DISPLAY` should show `<host-ip>:0`.
- Enable software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

### `No module named 'quadrotor_controller'`

Run Python from the `d:\Drones\` directory, or install the package:
```powershell
cd d:\Drones
pip install -e .
```

### WebSocket connection refused in browser

- Ensure `run_gazebo_bridge.py` is running before opening the browser.
- Check firewall isn't blocking ports **8080** (HTTP) or **8765** (WebSocket).

### UDP telemetry not updating Gazebo model

- Ensure `gzserver` is running and listening on port **9090** inside WSL.
- Verify WSL networking: `wsl hostname -I` should give a reachable IP from Windows.

### `pymavlink` import error

This is a non-critical warning. The simulation runs in **mock mode** without pymavlink — no real ArduPilot connection is made. Install it only if connecting to real hardware or SITL:
```powershell
pip install pymavlink
```

---

## 📄 License

This project is intended for academic research and educational use.  
Reference paper: *De Monte, P. & Lohmann, B. (2013). Trajectory tracking control for a quadrotor helicopter based on backstepping using a decoupling quaternion parametrization.*

---

*Built with Python 3, Gazebo 11, WSL2, and ❤️*
