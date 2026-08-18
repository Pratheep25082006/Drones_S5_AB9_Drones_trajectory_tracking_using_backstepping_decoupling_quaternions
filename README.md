<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/en/thumb/5/5d/Amrita-vishwa-vidyapeetham-color-logo.svg/320px-Amrita-vishwa-vidyapeetham-color-logo.svg.png" alt="Amrita Vishwa Vidyapeetham" width="300">
</p>

<h1 align="center">Quadrotor Backstepping Trajectory Tracking Control</h1>

<p align="center">
  <b>Real-Time Quaternion-Based Backstepping Controller with Gazebo 11 Simulation</b><br>
  Amrita Vishwa Vidyapeetham
</p>

---

## Team Members

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Kallam Pratheep Reddy | CB.SC.U4AIE24027 | kallampratheepreddy72@gmail.com |
| 2 | Chaithanya Gnambiar | CB.SC.U4AIE24012 | — |
| 3 | Surabhi Saha | CB.SC.U4AIE24055 | — |
| 4 | Anakha | CB.SC.U4AIE24003 | — |
| 5 | Mahendra Mula | CB.SC.U4AIE24035 | — |

---

## Abstract

This project implements a real-time **Backstepping Quaternion Trajectory Tracking Controller** for a quadrotor helicopter, simulated inside **Native Gazebo 11** via WSL2 Ubuntu on Windows. The control design is based on the decoupled quaternion parametrization proposed by De Monte & Lohmann (2013), which splits the attitude quaternion into a tilt component $q_{xy}$ and a heading component $q_z$. This decomposition allows the translational and yaw dynamics to be controlled independently, yielding a cleaner and more analytically tractable backstepping procedure. The system executes autonomous 3D flight trajectories — a Helical Spiral and a 3D Figure-8 Loop — while streaming live 6-DOF telemetry to a WebSocket-based browser visualizer at 50 Hz. An optional ArduPilot SITL/MAVLink bridge (`SITLBridge`) is included for hardware-in-the-loop extension.

---

## 1. Introduction

Autonomous quadrotor navigation requires simultaneously solving two coupled problems: generating a trajectory that is dynamically feasible and controlling the vehicle's attitude and position precisely enough to track it in real time.

**Why quaternions instead of Euler angles.**
Euler angle representations suffer from gimbal lock — a singularity where two rotational axes become aligned, causing a loss of one rotational degree of freedom and unstable controller behavior near that configuration. Unit quaternions represent orientation globally and without this singularity. This project uses quaternions throughout the attitude estimation and control pipeline.

**Why Backstepping.**
Backstepping is a recursive Lyapunov-based design method that exploits the cascaded structure of the quadrotor plant. At each stage of the cascade (position → velocity → thrust → attitude → angular rate), a virtual control law is derived that provably drives the corresponding error to zero, with a Lyapunov function constructed for the complete closed-loop system. This gives a constructive stability proof that purely PID-based approaches do not provide.

**Why the Decoupling Quaternion Parametrization.**
The key contribution of the base paper (De Monte & Lohmann, 2013) is a factorization $q = q_{xy} \otimes q_z$ that decouples the tilt degree of freedom (which affects translational dynamics) from the heading (yaw) degree of freedom, allowing two independent tracking control laws to be derived analytically within the backstepping framework.

---

## 2. Problem Statement

The objective of this project is to develop and simulate a real-time backstepping controller for a quadrotor that:

- Represents attitude as a unit quaternion, free from gimbal-lock singularities.
- Decomposes attitude into tilt ($q_{xy}$) and heading ($q_z$) to decouple translational and yaw dynamics.
- Executes pre-planned 3D trajectories (hover → helical spiral → figure-8 loop) in a physics simulator (Gazebo 11).
- Supports real-time custom target setpoints via a CLI utility and file-based IPC.
- Streams live 6-DOF telemetry at 50 Hz via WebSocket to a browser-based dashboard.
- Provides an optional MAVLink bridge for ArduPilot SITL or real hardware integration.

---

## 3. Base Paper

**De Monte, P. & Lohmann, B. (2013).** *"Trajectory Tracking Control for a Quadrotor Helicopter based on Backstepping using a Decoupling Quaternion Parametrization."* 21st Mediterranean Conference on Control & Automation (MED), Platanias-Chania, Crete, Greece, June 25–28, 2013. IEEE.

### Main Contributions of the Paper

- A novel factorization of the attitude quaternion into tilt ($q_{xy}$) and heading ($q_z$) components.
- Proof that this decomposition decouples the translational and yaw dynamics.
- A full backstepping trajectory tracking control law with analytical Lyapunov stability proof.
- Experimental validation on an AscTec Hummingbird at 50 Hz with a Vicon motion capture system.

### Relation to the Present Project

The present project implements the quaternion decomposition and backstepping control law from the paper in Python, and validates it in a Gazebo 11 3D simulation. The helix trajectory parameters (period times 6.25 s and 12.5 s) are taken directly from the paper's Fig. 4 experiment. Gains are re-tuned as scalar values for the simulation context.

---

## 4. Methodology

The system is implemented as four integrated layers, each independently testable.

### 4.1 Quaternion Attitude Parametrization

A unit quaternion $q \in \mathbb{R}^4$, $\|q\| = 1$, parameterizes the rotation from an inertial frame $\{I\}$ to the body-fixed frame $\{B\}$. Its time derivative is:

$$\dot{q} = \frac{1}{2} Q(q) \begin{bmatrix} \omega \\ 0 \end{bmatrix}$$

where $\omega = [\omega_x\ \omega_y\ \omega_z]^\top$ is the body angular velocity and $Q(q)$ is the quaternion product matrix (Eq. 4, base paper).

### 4.2 Decoupled Attitude Decomposition

Following Eq. (5)–(14) of the base paper, $q$ is factored as:

$$q = q_{xy} \otimes q_z$$

where:
- $q_{xy} = [q_x\ \ q_y\ \ 0\ \ q_p]^\top$ describes the **tilt** of the thrust vector (roll + pitch), with $q_p = \sqrt{q_3^2 + q_4^2}$
- $q_z = [0\ \ 0\ \ q_z\ \ q_w]^\top$ describes the **heading** (yaw)

The elements are recovered from the full quaternion $q = (q_1, q_2, q_3, q_4)^\top$ as:

$$q_p = \sqrt{q_3^2 + q_4^2}, \qquad
q_x = \frac{q_4 q_1 - q_3 q_2}{q_p}, \qquad
q_y = \frac{q_4 q_2 + q_3 q_1}{q_p}$$

$$q_w = \frac{|q_4|}{q_p}, \qquad q_z = \mathrm{sgn}(q_3 \cdot q_4)\frac{|q_3|}{q_p}$$

This decomposition is implemented in [`decompose_attitude`](quadrotor_controller/controller.py). The sign convention on $q_z$ eliminates the quaternion ambiguity and prevents the unwinding phenomenon (Bhat & Bernstein, 2000).

### 4.3 Translational Dynamics

The quadrotor translational dynamics in the inertial frame are (Eq. 25–26, base paper):

$$\dot{x} = v, \qquad m\dot{v} = f(v) + mg\hat{e}_3 + T$$

where $x = [x\ y\ z]^\top$ is position, $v$ is velocity, $m$ is mass, $g$ is gravity, and $T$ is the total thrust vector pointing along the body $+z$ axis. Drag $f(v)$ is omitted in the simulation.

### 4.4 Backstepping Position Control

The controller follows the cascaded Lyapunov backstepping procedure of Section III of the base paper.

**Step 1 — Position error and desired velocity:**

$$z_x = x - x_T, \qquad V_x = \tfrac{1}{2} z_x^\top z_x$$

$$\dot{V}_x = z_x^\top(v - \dot{x}_T) \implies v_d = \dot{x}_T + A_x z_x, \quad A_x < 0$$

**Step 2 — Velocity error and desired thrust vector:**

$$z_v = v - v_d, \qquad V_v = V_x + \tfrac{1}{2}z_v^\top z_v$$

$$T_d = m\bigl(-f(v) - g\hat{e}_3 + \dot{v}_d - z_x + A_v z_v\bigr), \quad A_v < 0$$

In code, this collapses to a desired acceleration:

$$\ddot{x}_{des} = \ddot{x}_T + k_{pos}(x_T - x) + k_{vel}(\dot{x}_T - v)$$

$$\vec{T} = m\,g\hat{e}_3 + m\,\ddot{x}_{des}$$

implemented in [`compute_command`](quadrotor_controller/controller.py).

**Horizontal acceleration safety cap** (simulation stability):

$$a_{xy} \leftarrow \frac{a_{xy}}{\|a_{xy}\|} \cdot \min\!\bigl(\|a_{xy}\|,\ 5.0\bigr) \text{ m/s}^2$$

**Thrust saturation:**

$$T = \max\!\bigl(0,\ \min\!\bigl(2.5\,mg,\ \|\vec{T}\|\bigr)\bigr)$$

### 4.5 Tilt Attitude Error (in $\mathbb{R}^3$)

Rather than computing a quaternion attitude error (which can suffer from the unwinding problem), the paper and this implementation use a **vectorial difference of thrust directions in $\mathbb{R}^3$** (Section III-A, base paper):

$$\hat{z}_{actual} = R(q)\,\hat{e}_3, \qquad \hat{z}_{desired} = \frac{\vec{T}}{\|\vec{T}\|}$$

$$\tau_{rp} = k_{tilt}\ (\hat{z}_{actual} \times \hat{z}_{desired}) - k_{rate}\ \omega_{rp}$$

The cross product selects the shortest rotational path and always rotates through the smaller angle, preventing unwanted large-angle rotations.

### 4.6 Independent Yaw Control

Since the heading dynamics are decoupled from translational dynamics (Eq. 24 and Section III-B of the base paper), yaw is controlled independently:

$$\psi_{error} = \mathrm{wrap}(\psi_{ref} - \psi_{actual}),$$

$$\tau_{yaw} = k_{yaw}\,\psi_{error} - k_{rate}\,\omega_z$$

where $\mathrm{wrap}(\cdot)$ maps the angle to $(-\pi, \pi]$. The closed-loop heading dynamics are:

$$\dot{z}_\psi = a_\psi\, z_\psi + z_{\omega_z}$$
$$\dot{z}_{\omega_z} = a_{\omega_z}\, z_{\omega_z} - z_\psi$$

which is asymptotically stable for $a_\psi < 0$, $a_{\omega_z} < 0$ (Eq. 53–54, base paper).

### 4.7 Full Control Command

At every 20 ms step the controller outputs:

```
thrust, (τ_roll, τ_pitch, τ_yaw) = controller.compute_command(state, reference, dt=0.02)
```

### 4.8 Trajectory Generation

Two reference trajectories are implemented, with parameters matching the base paper's experiments:

**Helical Spiral** (period from paper: $T_{xy} = 6.25$ s, $T_z = 12.5$ s):

$$\omega_{xy} = \frac{2\pi}{6.25},\quad \omega_z = \frac{2\pi}{12.5}$$

$$x_T = 2\sin(\omega_{xy}\,t),\quad y_T = 2\cos(\omega_{xy}\,t),\quad z_T = 1.5 + \sin(\omega_z\,t)$$

**3D Figure-8 (Lemniscate):**

$$\omega = \frac{2\pi}{9},\quad x_T = 2.5\sin(\omega t),\quad y_T = 2.5\sin(2\omega t),\quad z_T = 2.2 + 0.6\cos(\omega t)$$

Full position, velocity, and acceleration references (and their derivatives) are passed to the backstepping controller at each step.

### 4.9 2nd-Order Reference Filter (Bridge Mode)

When running via the WebSocket bridge, a smooth 2nd-order reference filter (time constant $\tau = 0.8$ s) generates $C^2$-continuous position references from a step-changing setpoint, as required by the backstepping derivation:

$$\ddot{x}_{ref} = \frac{x_{target} - x_{ref} - 2\tau\,\dot{x}_{ref}}{\tau^2}$$

$$\dot{x}_{ref} \leftarrow \dot{x}_{ref} + \ddot{x}_{ref}\,\Delta t, \qquad x_{ref} \leftarrow x_{ref} + \dot{x}_{ref}\,\Delta t$$

---

## 5. Controller Gains

| Parameter | Symbol | Value | Description |
|---|---|---:|---|
| Mass | $m$ | 1.5 kg | Vehicle mass |
| Gravity | $g$ | 9.81 m/s² | Gravitational acceleration |
| Position gain | $k_{pos}$ ($A_x$) | 2.0 | Drives position error → 0 |
| Velocity gain | $k_{vel}$ ($A_v$) | 1.5 | Drives velocity error → 0 |
| Tilt gain | $k_{tilt}$ ($A_t$) | 3.5 | Roll/pitch attitude correction |
| Yaw gain | $k_{yaw}$ ($a_\psi$) | 2.5 | Heading error correction |
| Rate damping | $k_{rate}$ ($A_r$) | 0.8 | Angular rate damping |

### Safety Limits

| Limit | Value |
|---|---|
| Max horizontal acceleration | 5.0 m/s² |
| Max thrust | $2.5 \times mg \approx 36.8$ N |
| Minimum altitude | 0.2 m |

---

## 6. System Architecture

```
Windows Host
│
├── run_gazebo_simulation.py     ← Main entry point (50 Hz control loop)
│     ├── BacksteppingController ← Thrust + torque from backstepping law
│     ├── SimulationRunner       ← Kinematics integration + UDP pose stream
│     └── file_listener_loop     ← Monitors target.txt for setpoint updates
│
├── run_gazebo_bridge.py         ← WebSocket bridge + HTTP file server
│     ├── WebSocket :8765        ← JSON telemetry → browser
│     └── HTTP :8080             ← Serves gazebo_visualizer.html
│
└── set_target.py                ← CLI utility: write X Y Z to target.txt

WSL2 Ubuntu
│
├── gzserver / gzclient          ← Gazebo 11 physics + GUI (via VcXsrv X11)
└── UDP Listener :9090           ← Receives 6-DOF pose → gz model update
```

---

## 7. Project Structure

```
d:\Drones\
│
├── quadrotor_controller/          # Core Python controller package
│   ├── __init__.py                # Public API
│   ├── controller.py              # BacksteppingController + quaternion math
│   └── sitl_bridge.py             # SITLBridge — MAVLink & simulator bridge
│
├── models/
│   └── quadrotor.world            # Gazebo 11 SDF world
│
├── tests/
│   ├── test_controller.py         # Unit tests — BacksteppingController
│   └── test_sitl_bridge.py        # Unit tests — SITLBridge
│
├── run_gazebo_simulation.py       # ⭐ Main simulation runner (Windows)
├── run_gazebo_bridge.py           # WebSocket telemetry bridge server
├── gazebo_visualizer.html         # Real-time 3D web telemetry dashboard
├── set_target.py                  # CLI target setter utility
├── launch_gazebo_wsl.py           # Launches Gazebo inside WSL2
├── run_gazebo.sh                  # Bash launcher (inside WSL Ubuntu)
│
├── START_GAZEBO_GUI.bat           # 1-click Windows launcher
├── launch_web_visualizer.bat      # 1-click web visualizer launcher
│
├── GAZEBO_INSTALLATION_GUIDE.md   # Full Gazebo 11 + WSL2 setup guide
└── Trajectory_tracking_control... # Reference research paper (PDF)
```

---

## 8. Mathematical Formulation Summary

| Symbol | Definition |
|---|---|
| $q = (w, x, y, z)^\top$ | Unit quaternion, $\|q\| = 1$ |
| $q_{xy}$ | Tilt quaternion — encodes thrust direction |
| $q_z$ | Heading quaternion — encodes yaw |
| $q = q_{xy} \otimes q_z$ | Decoupled parametrization (base paper Eq. 5) |
| $z_x = x - x_T$ | Position tracking error |
| $z_v = v - v_d$ | Velocity error |
| $z_t = T - T_d$ | Thrust vector error |
| $\hat{z}_{act} \times \hat{z}_{des}$ | Tilt attitude error in $\mathbb{R}^3$ |
| $\psi_{err} = \mathrm{wrap}(\psi_{ref} - \psi)$ | Yaw error |
| $V = V_x + V_v + V_t + V_\omega$ | Composite Lyapunov function |

The complete closed-loop Lyapunov derivative satisfies:

$$\dot{V} = z_x^\top A_x z_x + z_v^\top A_v z_v + z_t^\top A_t z_t + z_r^\top A_r z_r \leq 0$$

with $A_x, A_v, A_t, A_r < 0$, proving asymptotic stability (Eq. 46, base paper).

---

## 9. Prerequisites & Installation

### System Requirements

| Component | Requirement |
|---|---|
| OS | Windows 10 / 11 with WSL2 |
| WSL2 | Ubuntu 22.04 LTS |
| Python | 3.9+ (Windows side) |
| X11 Server | VcXsrv |
| Simulator | Gazebo 11 (via Pixi in WSL) |

### Step 1 — Install VcXsrv

```powershell
winget install marha.VcXsrv
```

Launch **XLaunch**: Multiple Windows → Display `0` → ✅ Disable access control.

### Step 2 — Install WSL2 Ubuntu

```powershell
wsl --install -d Ubuntu
```

### Step 3 — Install Gazebo 11 via Pixi (inside WSL)

```bash
curl -fsSL https://pixi.sh/install.sh | bash && source ~/.bashrc
pixi global install gazebo
```

### Step 4 — Install OpenGL & X11 libs (WSL)

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential mesa-utils libgl1-mesa-dri \
  libgl1-mesa-glx libglx-mesa0 libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4
```

### Step 5 — Install Python dependencies (Windows)

```powershell
cd d:\Drones
python -m venv .venv
.venv\Scripts\activate
pip install pymavlink websockets
```

> `pymavlink` is optional — the simulation runs in pure-Python mock mode without it.

See [`GAZEBO_INSTALLATION_GUIDE.md`](GAZEBO_INSTALLATION_GUIDE.md) for the full setup walkthrough.

---

## 10. How to Run

### Option A — 1-Click GUI Launch

Double-click **`START_GAZEBO_GUI.bat`** — automatically starts VcXsrv, launches Gazebo 11, and runs the controller.

### Option B — Manual Launch

**Terminal 1 — Start simulation:**
```powershell
cd d:\Drones
python run_gazebo_simulation.py
```

**Terminal 2 — Start web visualizer (optional):**
```powershell
python run_gazebo_bridge.py
```
Open `http://localhost:8080` in your browser.

### Option C — Custom Target at Launch

```powershell
python run_gazebo_simulation.py 5 5 10
```
Flies immediately to X=5 m, Y=5 m, Z=10 m.

---

## 11. Flight Modes

### Trajectory Mode (Auto — 40-second repeating cycle)

| Phase | Time | Description |
|---|---|---|
| Hover | 0 – 4 s | Stabilize at (0, 0, 2 m) |
| Helical Spiral | 4 – 22 s | Circular orbit $r=2$ m, altitude oscillation (matches paper Fig. 4) |
| 3D Figure-8 | 22 – 40 s | Lemniscate of Bernoulli in 3D space |

### Target Setpoint Mode

Fly to and hold a custom (X, Y, Z) coordinate — activated by `set_target.py` or CLI argument.

---

## 12. Setting a Custom Target

```powershell
# Fly to X=5m, Y=5m, Z=10m
python set_target.py 5 5 10

# Fly to negative coordinates
python set_target.py -3 2 5

# Resume automatic trajectory
python set_target.py auto
```

The utility writes to `target.txt`; the simulation polls it every 500 ms. No restart required.

---

## 13. API Reference

### `BacksteppingController`

```python
from quadrotor_controller import BacksteppingController, ControllerState, ControllerReference

controller = BacksteppingController(mass=1.5, gravity=9.81)

state = ControllerState(
    position=(x, y, z),
    velocity=(vx, vy, vz),
    quaternion=(w, x, y, z),    # unit quaternion
    angular_rate=(wx, wy, wz),
)

reference = ControllerReference(
    position=(rx, ry, rz),
    velocity=(rvx, rvy, rvz),
    acceleration=(rax, ray, raz),
    heading=0.0,                 # yaw in radians
)

thrust, torques = controller.compute_command(state, reference, dt=0.02)
# thrust  → scalar N
# torques → (roll_torque, pitch_torque, yaw_torque)
```

### Quaternion Utilities

```python
from quadrotor_controller import (
    quaternion_from_euler,   # (roll, pitch, yaw) → (w, x, y, z)
    normalize_quaternion,    # q → unit quaternion
    decompose_attitude,      # q → (q_xy, q_z) tilt/heading split
)
```

### `SITLBridge` (Optional MAVLink)

```python
from quadrotor_controller import SITLBridge

bridge = SITLBridge(connection_string="udp:127.0.0.1:14550")
bridge.connect(timeout=10.0)
bridge.set_mode("GUIDED")
bridge.arm_vehicle(arm=True)
target = bridge.build_attitude_target(thrust=14.7, roll=0.0, pitch=0.05, yaw=0.0, yaw_rate=0.0)
bridge.send_attitude_target(target)
bridge.disconnect()
```

---

## 14. Running Tests

```powershell
cd d:\Drones
python -m pytest tests/ -v
```

| Test | Description |
|---|---|
| `test_normalize_quaternion` | Verifies $\|q\| = 1$ after normalization |
| `test_decompose_attitude_recombines_to_original_quaternion` | Validates $q_{xy} \otimes q_z = q$ (Eq. 5, base paper) |
| `test_controller_returns_positive_thrust_and_three_torques` | Output shape, type, and sign correctness |

---

## 15. Key Observations

1. The quaternion decoupling $q = q_{xy} \otimes q_z$ allows translational and yaw dynamics to be controlled by two completely independent laws, each with its own Lyapunov stability proof.
2. Computing tilt error as $\hat{z}_{actual} \times \hat{z}_{desired}$ in $\mathbb{R}^3$ guarantees the shortest-angle rotation and avoids the unwinding phenomenon without any sign-flip logic on the quaternion.
3. The helical trajectory period times ($T_{xy} = 6.25$ s, $T_z = 12.5$ s) are taken directly from the paper's experimental flight in Fig. 4 and reproduced exactly in the simulation.
4. Scalar gains are used in place of the paper's matrix gains ($A_x = -I$, $A_v = -3I$, $A_t = -8I$, $A_r = -12I$) — a standard simplification when hardware inertia parameters are not available.
5. The 50 Hz control-loop rate matches the telemetry rate reported in the paper's experimental setup (Vicon data at 50 Hz to the quadrotor via radio link).

---

## 16. Limitations

- Aerodynamic drag $f(v)$ is not modeled (set to zero in the simulation).
- The inertia tensor $J$ and gyroscopic coupling term $-J\omega \times \omega$ are not modeled.
- The dynamic thrust extension ($\ddot{T} = u_T$, Eq. 31 of the paper) is omitted; thrust is computed algebraically.
- Full 4-step backstepping cascade (position → velocity → thrust error → angular velocity error) is collapsed to a 2-step proportional-derivative computation.
- No real sensor noise or IMU model is included.

---

## 17. Future Work

- Add full inertia tensor and gyroscopic feedforward term.
- Implement the complete 4-step backstepping Lyapunov cascade with matrix gains.
- Include aerodynamic drag model $f(v)$ identified from flight data.
- Extend to adaptive gain tuning to handle unknown or time-varying parameters.
- Port the controller to ArduPilot or PX4 via the existing `SITLBridge` MAVLink interface.
- Add realistic IMU sensor noise and an attitude estimator (e.g. Mahony filter).
- Implement full ROS2 integration for SLAM and obstacle avoidance.

---

## 18. Conclusion

This project implements and validates, in a Gazebo 11 simulation, a complete real-time backstepping trajectory tracking controller for a quadrotor, based on the decoupled quaternion parametrization of De Monte & Lohmann (2013). The core design choices — unit quaternion attitude representation, tilt/heading decomposition ($q = q_{xy} \otimes q_z$), and $\mathbb{R}^3$ cross-product attitude error — are reproduced faithfully from the paper and confirmed to produce stable trajectory tracking in the Helical Spiral and Figure-8 maneuvers. The helix trajectory period parameters match the paper's experimental results exactly. The WebSocket telemetry bridge and live browser visualizer provide real-time monitoring of all state variables, and the `set_target.py` utility allows interactive re-tasking of the drone without restarting the simulation.

---

## 19. References

1. **Base Paper** — P. De Monte and B. Lohmann, *"Trajectory Tracking Control for a Quadrotor Helicopter based on Backstepping using a Decoupling Quaternion Parametrization,"* 21st Mediterranean Conference on Control & Automation (MED), Crete, Greece, June 2013. IEEE.

2. M. Krstic, I. Kanellakopoulos, and P. Kokotovic, *Nonlinear and Adaptive Control Design*, John Wiley & Sons, 1995. *(Backstepping foundations)*

3. S. P. Bhat and D. S. Bernstein, *"A topological obstruction to continuous global stabilization of rotational motion and the unwinding phenomenon,"* Systems & Control Letters, vol. 39, pp. 63–70, 2000.

4. O. Fritsch, P. De Monte, M. Buhl, and B. Lohmann, *"Quasi-static feedback linearization for the translational dynamics of a quadrotor helicopter,"* Proceedings of the American Control Conference, 2012.

5. T. Hamel, R. Mahony, R. Lozano, and J. Ostrowski, *"Dynamic modelling and configuration stabilization for an X4-Flyer,"* Proceedings of the 15th IFAC World Congress, 2002.
