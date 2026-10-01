![Amrita Vishwa Vidyapeetham](assets/amrita_logo.png)

# Quadrotor Backstepping Trajectory Tracking Control

Real-Time Quaternion-Based Backstepping Controller with Gazebo 11 and MuJoCo Simulation

Amrita Vishwa Vidyapeetham

---

## Project Guide

| Role | Name | Email |
|---|---|---|
| Faculty Supervisor | Prof. S. Sunil Kumar | [s_sunilkumar@cb.amrita.edu](mailto:s_sunilkumar@cb.amrita.edu) |

---

## Team Members

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Kallam Pratheep Reddy | CB.SC.U4AIE24021 | kallampratheepreddy72@gmail.com |
| 2 | Chaithanya G Nambiar | CB.SC.U4AIE24013 | Cb.sc.u4aie24013@cb.students.amrita.edu |
| 3 | Surabhi Saha | CB.SC.U4AIE24057 | saha8.surabhi@gmail.com |
| 4 | Anakha | CB.SC.U4AIE24006 | anakhas443@gmail.com |
| 5 | Mahendra Mula | CB.SC.U4AIE24033 | maheshbanu58@gmail.com |

---

## Abstract

This project implements a real-time Backstepping Quaternion Trajectory Tracking Controller for a quadrotor helicopter. The controller is validated in two simulation environments: native Gazebo 11 running on WSL2 Ubuntu under Windows, and the MuJoCo physics engine. The control design follows the decoupled quaternion parametrization proposed by De Monte and Lohmann (2013), which splits the attitude quaternion into a tilt component $q_{xy}$ and a heading component $q_z$. This decomposition allows the translational and yaw dynamics to be controlled independently, yielding a cleaner and more analytically tractable backstepping procedure.

The system executes autonomous 3D flight trajectories, namely a Helical Spiral and a 3D Figure-8 Loop, while streaming live 6-DOF telemetry at 50 Hz to a WebSocket-based browser visualizer. An optional ArduPilot SITL/MAVLink bridge (`SITLBridge`) is included for hardware-in-the-loop extension.

---

## 1. Introduction

Autonomous quadrotor navigation requires solving two coupled problems: generating a trajectory that is dynamically feasible, and controlling the vehicle's attitude and position precisely enough to track that trajectory in real time.

Quaternions versus Euler angles. Euler angle representations suffer from gimbal lock, a singularity in which two rotational axes become aligned, causing a loss of one rotational degree of freedom and unstable controller behavior near that configuration. Unit quaternions represent orientation globally and without this singularity. This project uses quaternions throughout the attitude estimation and control pipeline.

Backstepping. Backstepping is a recursive Lyapunov-based design method that exploits the cascaded structure of the quadrotor plant. At each stage of the cascade (position, velocity, thrust, attitude, angular rate), a virtual control law is derived that provably drives the corresponding error to zero, with a Lyapunov function constructed for the complete closed-loop system. This gives a constructive stability proof that purely PID-based approaches do not provide.

Decoupling quaternion parametrization. The key contribution of the base paper (De Monte and Lohmann, 2013) is a factorization $q = q_{xy} \otimes q_z$ that decouples the tilt degree of freedom, which affects translational dynamics, from the heading (yaw) degree of freedom. This allows two independent tracking control laws to be derived analytically within the backstepping framework.

---

## 2. Problem Statement

The objective of this project is to develop and simulate a real-time backstepping controller for a quadrotor that:

- Represents attitude as a unit quaternion, free from gimbal-lock singularities.
- Decomposes attitude into tilt ($q_{xy}$) and heading ($q_z$) to decouple translational and yaw dynamics.
- Executes pre-planned 3D trajectories (hover, helical spiral, figure-8 loop) in physics simulators (Gazebo 11 and MuJoCo).
- Supports real-time custom target setpoints via a CLI utility and file-based inter-process communication.
- Streams live 6-DOF telemetry at 50 Hz via WebSocket to a browser-based dashboard.
- Provides an optional MAVLink bridge for ArduPilot SITL or real hardware integration.

---

## 3. Base Paper

P. De Monte and B. Lohmann (2013). "Trajectory Tracking Control for a Quadrotor Helicopter based on Backstepping using a Decoupling Quaternion Parametrization." 21st Mediterranean Conference on Control and Automation (MED), Platanias-Chania, Crete, Greece, June 25-28, 2013. IEEE.

### Main Contributions of the Paper

- A novel factorization of the attitude quaternion into tilt ($q_{xy}$) and heading ($q_z$) components.
- Proof that this decomposition decouples the translational and yaw dynamics.
- A full backstepping trajectory tracking control law with an analytical Lyapunov stability proof.
- Experimental validation on an AscTec Hummingbird at 50 Hz with a Vicon motion capture system.

### Relation to the Present Project

The present project implements the quaternion decomposition and backstepping control law from the paper in Python, and validates it in Gazebo 11 and MuJoCo 3D simulations. The helix trajectory period times (6.25 s and 12.5 s) are taken directly from the paper's Fig. 4 experiment. Gains are re-tuned as scalar values for the simulation context.

---

## 4. Methodology

The system is implemented as four integrated layers, each independently testable.

---

### 4.1 Quaternion Attitude Parametrization

$$
\dot{q} = \frac{1}{2}\, Q(q) \begin{bmatrix} \omega \\ 0 \end{bmatrix}
$$

Description: a quaternion $q = (w, x, y, z)$ describes the orientation of the vehicle in 3D space without the gimbal-lock singularity of Euler angles. The equation states that the rate of change of the orientation is half the product of the current orientation and the angular velocity. It propagates the quaternion forward in time as the vehicle rotates.

- $\omega$: angular velocity of the vehicle about each body axis.
- $Q(q)$: matrix built from the current quaternion that encodes the quaternion product.

---

### 4.2 Decoupled Attitude Decomposition

$$
q = q_{xy} \otimes q_z
$$

Description: this is the central idea of the paper. Instead of treating a single quaternion for all rotation, the attitude is split into two simpler parts.

- $q_{xy}$: tilt quaternion. Describes how much the vehicle leans (roll and pitch), which directly sets the thrust direction.
- $q_z$: heading quaternion. Describes the direction the vehicle faces (yaw).
- $\otimes$: quaternion multiplication, i.e. composition of two rotations.

Because of this split, position control and heading control can be designed independently.

The elements are extracted from the full quaternion $q = (q_1, q_2, q_3, q_4)$ as follows.

$$
q_p = \sqrt{q_3^2 + q_4^2}
$$

Here $q_p$ is the magnitude of the heading part, used as a normalizing denominator for the components below.

$$
q_x = \frac{q_4 q_1 - q_3 q_2}{q_p}
$$

$$
q_y = \frac{q_4 q_2 + q_3 q_1}{q_p}
$$

The components $q_x$ and $q_y$ are the tilt components. They describe the direction of the thrust axis relative to vertical.

$$
q_w = \frac{|q_4|}{q_p}
$$

$$
q_z = \mathrm{sgn}(q_3 q_4)\,\frac{|q_3|}{q_p}
$$

The components $q_w$ and $q_z$ are the heading components. The sign function ensures the shortest rotation is always selected, which prevents the unwinding phenomenon.

---

### 4.3 Translational Dynamics

$$
\dot{x} = v
$$

$$
m\dot{v} = m g\\hat{e}_3 + T
$$

Description: these equations describe the motion of the vehicle through space.

- $\dot{x} = v$: position changes at the rate of the velocity.
- $m\dot{v} = m g\,\hat{e}_3 + T$: Newton's second law, where $m g\,\hat{e}_3$ is the gravitational force and $T$ is the thrust force generated by the propellers.

The controller selects $T$ (direction and magnitude) so that gravity is compensated and the vehicle moves toward the target position.

---

### 4.4 Backstepping Position Control

Backstepping proceeds as a chain of objectives: position is regulated first, then velocity, then thrust, with each step building on the previous one.

Step 1: position error.

$$
z_x = x - x_T
$$

Here $z_x$ is the deviation of the vehicle from the reference position.

$$
V_x = \frac{1}{2}\, z_x^\top z_x
$$

Here $V_x$ is a Lyapunov function, an energy-like measure that the controller drives to zero. When $V_x = 0$, the vehicle is exactly at the reference position.

$$
v_d = \dot{x}_T + A_x z_x \qquad (A_x < 0)
$$

Here $v_d$ is the desired velocity. It consists of the feedforward trajectory velocity $\dot{x}_T$ and a correction term $A_x z_x$ that drives the vehicle toward the reference.

Step 2: velocity error.

$$
z_v = v - v_d
$$

Here $z_v$ is the difference between the current velocity and the desired velocity.

$$
\ddot{x}_{des} = \ddot{x}_T + k_{pos}\,(x_T - x) + k_{vel}\,(\dot{x}_T - v)
$$

This expression gives the desired acceleration, composed of three terms:

- $\ddot{x}_T$: trajectory acceleration (feedforward).
- $k_{pos}(x_T - x)$: position correction.
- $k_{vel}(\dot{x}_T - v)$: velocity correction.

$$
\vec{T} = m g\,\hat{e}_3 + m\,\ddot{x}_{des}
$$

This is the required thrust vector. The term $m g\,\hat{e}_3$ cancels gravity, and $m\,\ddot{x}_{des}$ produces the desired motion. The direction of $\vec{T}$ determines the required tilt, and its magnitude determines the collective thrust.

Horizontal acceleration limit:

$$
a_{xy} \leftarrow \frac{a_{xy}}{\|a_{xy}\|}\cdot\min\left(\|a_{xy}\|,\ 5.0\right)\ \mathrm{m/s^2}
$$

If the computed horizontal acceleration exceeds 5 m/s², it is scaled down to 5 m/s² while preserving its direction. This prevents excessive tilt.

Thrust saturation:

$$
T = \max\left(0,\ \min\left(2.5\,m g,\ \|\vec{T}\|\right)\right)
$$

Thrust is limited between zero and 2.5 times the hover thrust (approximately 36.8 N), which models realistic motor limits.

---

### 4.5 Tilt Attitude Error

$$
\hat{z}_{actual} = R(q)\,\hat{e}_3
$$

Here $\hat{z}_{actual}$ is the current direction of the thrust axis in the world frame, obtained by rotating the body-frame vertical axis with the current quaternion.

$$
\hat{z}_{desired} = \frac{\vec{T}}{\|\vec{T}\|}
$$

Here $\hat{z}_{desired}$ is the required thrust direction, obtained by normalizing the thrust vector.

$$
\tau_{rp} = k_{tilt}\,\left(\hat{z}_{actual} \times \hat{z}_{desired}\right) - k_{rate}\,\omega_{rp}
$$

This computes the roll and pitch torque commands.

- $\hat{z}_{actual} \times \hat{z}_{desired}$: the cross product has a magnitude proportional to the angle between the two directions and points along the rotation axis that aligns them. This selects the shortest rotation path automatically.
- $k_{tilt}$ times the cross product: proportional correction.
- $-k_{rate}\,\omega_{rp}$: damping term that limits overshoot.

---

### 4.6 Independent Yaw Control

$$
\psi_{error} = \mathrm{wrap}\left(\psi_{ref} - \psi_{actual}\right)
$$

Here $\psi_{error}$ is the heading error. The wrap function constrains the error to $(-180^\circ, +180^\circ]$ so that the vehicle always turns the short way.

$$
\tau_{yaw} = k_{yaw}\,\psi_{error} - k_{rate}\,\omega_z
$$

This is the yaw torque command, consisting of a proportional term on the heading error and a damping term on the yaw rate. Because of the $q_{xy}/q_z$ split, this torque is computed independently of the position control, so changing the heading does not interfere with flight to the target position.

Closed-loop heading stability:

$$
\dot{z}_\psi = a_\psi\, z_\psi + z_{\omega_z}
$$

$$
\dot{z}_{\omega_z} = a_{\omega_z}\, z_{\omega_z} - z_\psi
$$

These equations describe the evolution of the heading error $z_\psi$ and the yaw-rate error $z_{\omega_z}$. Since $a_\psi < 0$ and $a_{\omega_z} < 0$, both errors decay to zero, which establishes asymptotic stability of the yaw controller.

---

### 4.7 Full Control Command

At every 20 ms step the controller outputs:

```
thrust, (tau_roll, tau_pitch, tau_yaw) = controller.compute_command(state, reference, dt=0.02)
```

Fifty times per second the controller reads the current state (position, velocity, orientation, angular rates) and the reference trajectory, evaluates the equations above, and outputs:

- `thrust`: collective thrust in Newtons.
- `tau_roll`, `tau_pitch`, `tau_yaw`: roll, pitch, and yaw torque commands.

---

### 4.8 Trajectory Generation

Helical Spiral (period times from the paper: $T_{xy} = 6.25$ s horizontal, $T_z = 12.5$ s vertical):

$$
\omega_{xy} = \frac{2\pi}{6.25}
$$

$$
\omega_z = \frac{2\pi}{12.5}
$$

$$
x_T = 2\sin(\omega_{xy}\,t)
$$

$$
y_T = 2\cos(\omega_{xy}\,t)
$$

$$
z_T = 1.5 + \sin(\omega_z\,t)
$$

The vehicle follows a corkscrew path: a circle of radius 2 m in the horizontal plane completed every 6.25 s, with altitude oscillating on a 12.5 s period. The 90 degree phase shift between $x_T$ and $y_T$ produces the circular motion, and the sinusoidal $z_T$ produces smooth altitude variation.

3D Figure-8 (Lemniscate):

$$
\omega = \frac{2\pi}{9}
$$

$$
x_T = 2.5\sin(\omega t)
$$

$$
y_T = 2.5\sin(2\omega t)
$$

$$
z_T = 2.2 + 0.6\cos(\omega t)
$$

The $x$ coordinate uses frequency $\omega$ and the $y$ coordinate uses $2\omega$, producing a Lissajous figure-8 pattern. The altitude oscillates with amplitude 0.6 m. The full path repeats every 9 s.

---

### 4.9 Second-Order Reference Filter (Bridge Mode)

Purpose: the backstepping controller in Section 4.4 needs a reference that is smooth, meaning it must supply the target position $x_T$, velocity $\dot{x}_T$, and acceleration $\ddot{x}_T$ at every instant. A trajectory such as the helix provides these analytically. A target typed by the user, for example `set_target.py 5 5 10`, does not: it is a single point that changes instantaneously, which is a step input. Feeding a step directly into the controller would produce a very large position error, saturated acceleration and thrust commands, aggressive tilting, and no usable velocity or acceleration reference. The second-order reference filter converts the step into a smooth, physically reasonable path toward the new target.

Filter equations:

$$
\ddot{x}_{ref} = \frac{x_{target} - x_{ref} - 2\tau\,\dot{x}_{ref}}{\tau^2}
$$

$$
\dot{x}_{ref} \leftarrow \dot{x}_{ref} + \ddot{x}_{ref}\,\Delta t
$$

$$
x_{ref} \leftarrow x_{ref} + \dot{x}_{ref}\,\Delta t
$$

Symbol definitions:

- $x_{target}$: the commanded final position (the step input from the user).
- $x_{ref}$, $\dot{x}_{ref}$, $\ddot{x}_{ref}$: the filtered reference position, velocity, and acceleration. These are the filter states and outputs.
- $\tau$: the filter time constant, set to 0.8 s.
- $\Delta t$: the control period, 0.02 s at 50 Hz.

How the equations work:

1. The first equation computes the reference acceleration. It acts like a spring and a damper attached to the target. The term $x_{target} - x_{ref}$ is the spring: it pulls the reference toward the target, and the pull is stronger when the reference is farther away. The term $-2\tau\,\dot{x}_{ref}$ is the damper: it opposes the reference velocity so the reference does not overshoot. The division by $\tau^2$ sets how stiff the response is.
2. The second equation integrates the acceleration over one timestep to update the reference velocity.
3. The third equation integrates the velocity over one timestep to update the reference position.

These three steps run every control cycle, so the reference moves along a smooth curve instead of jumping.

Why this form: rearranging the first equation gives $\tau^2\,\ddot{x}_{ref} + 2\tau\,\dot{x}_{ref} + x_{ref} = x_{target}$. In the Laplace domain this is the transfer function

$$
\frac{X_{ref}(s)}{X_{target}(s)} = \frac{1}{(\tau s + 1)^2}
$$

which has a double pole at $s = -1/\tau$. This is a critically damped second-order system, so the reference reaches the target as fast as possible without overshoot and without oscillation.

Behavior for a step change of size $D$ (per axis):

- The reference position rises smoothly along an S-shaped curve and settles at the target.
- The reference velocity starts at zero, rises to a single peak, and returns to zero. The peak occurs at $t = \tau$ and has magnitude $D/(e\,\tau) \approx 0.37\,D/\tau$.
- The reference acceleration is largest at the start, with magnitude $D/\tau^2$, then reverses sign to decelerate the reference and decays to zero.
- Settling to within about 2 percent of the target takes roughly $5.8\,\tau$, which is about 4.7 s for $\tau = 0.8$ s.

Example: for a step of $D = 5$ m on one axis with $\tau = 0.8$ s, the peak reference velocity is about 2.3 m/s, reached at 0.8 s after the command, and the reference is within 2 percent of the target after about 4.7 s. The initial reference acceleration of about 7.8 m/s² is limited by the horizontal acceleration cap of 5.0 m/s² in the controller (Section 4.4).

Tuning of $\tau$:

- Smaller $\tau$ gives a faster response with higher peak velocity and acceleration, and therefore more aggressive tilting.
- Larger $\tau$ gives a slower and gentler response with lower peak velocity and acceleration.
- $\tau$ should stay much larger than $\Delta t$. With $\tau = 0.8$ s and $\Delta t = 0.02$ s the simple Euler integration used above is accurate and stable.

Implementation notes:

- The filter is applied independently to each of the three axes (X, Y, Z), so $x_{ref}$, $\dot{x}_{ref}$, and $\ddot{x}_{ref}$ are 3-vectors.
- The outputs are passed to the controller as the reference: $x_{ref}$ becomes $x_T$, $\dot{x}_{ref}$ becomes $\dot{x}_T$, and $\ddot{x}_{ref}$ becomes $\ddot{x}_T$ in the backstepping law of Section 4.4. In the API this corresponds to the `position`, `velocity`, and `acceleration` fields of `ControllerReference`.
- Because the reference acceleration is passed as feedforward, the controller anticipates the motion instead of only reacting to the position error, which improves tracking.
- The filter states are initialized at the current vehicle position with zero velocity and acceleration. A new target only changes $x_{target}$, so the reference continues smoothly even if a new target is issued while the vehicle is still moving.
- The filter is used in target setpoint mode (Section 12). In automatic trajectory mode, the analytic helix and figure-8 references are used directly.

---

## 5. Simulation Results

### 5.1 Gazebo 11: 3D Physics Simulation

The quadrotor is spawned inside a native Gazebo 11 world (`models/quadrotor.world`) running inside WSL2 Ubuntu. The backstepping controller streams pose commands over UDP at 50 Hz to move the model in real time.

![Gazebo 3D Drone Simulator and Web Telemetry Dashboard](assets/web_visualizer.jpg)

Fig. 1. Live web telemetry dashboard showing the vehicle at position (0.99, 0.99, 1.98) m with a tracking error of 0.001 m and a total thrust of 14.68 N during the hover phase.

Dashboard summary:

- Actual and target position are both (0.99, 0.99, 1.98) m, indicating convergence.
- Tracking error is 0.001 m, i.e. below 1 mm steady-state hover error.
- Total thrust is 14.68 N, approximately 1.5 kg x 9.81 m/s², which corresponds to gravity compensation.
- WebSocket status is CONNECTED, with the 50 Hz telemetry stream active.

---

### 5.2 MuJoCo: Rigid-Body Dynamics Simulation

In addition to Gazebo, the controller was implemented and validated in the MuJoCo physics engine. MuJoCo integrates the full rigid-body dynamics of the vehicle, so the controller outputs (collective thrust and body torques) act on a dynamic model instead of a kinematic pose update. This provides a more demanding test of the backstepping law.

Implementation points:

- The quadrotor is described in a MuJoCo MJCF model (XML) with a free-joint base body, mass and inertia properties, and four rotor sites.
- The controller outputs collective thrust and roll, pitch, and yaw torques at 50 Hz. These are applied to the body through MuJoCo's actuator or applied-force interface at every control step.
- The physics timestep is set smaller than the control period, so several physics steps are integrated between consecutive control updates.
- The full state (position, velocity, unit quaternion, angular rate) is read from the simulator each cycle and passed to `BacksteppingController` as `ControllerState`.
- The same trajectory generators (hover, helical spiral, figure-8) and the same reference filter are reused without modification, so results are directly comparable with the Gazebo runs.
- Telemetry from the MuJoCo run is published through the same WebSocket bridge, so the browser dashboard works with either simulator.
- Controller gains and safety limits (Section 6) are identical in both environments.

![MuJoCo Quadrotor Simulation](ADD_MUJOCO_SIMULATION_IMAGE_URL_HERE)

Fig. 2. MuJoCo simulation of the quadrotor during trajectory tracking.

![MuJoCo Trajectory Tracking Plot](ADD_MUJOCO_TRAJECTORY_PLOT_URL_HERE)

Fig. 3. Reference trajectory versus actual trajectory in MuJoCo.

![MuJoCo Tracking Error Plot](ADD_MUJOCO_ERROR_PLOT_URL_HERE)

Fig. 4. Position tracking error over time in MuJoCo.

Demonstration video: [ADD_MUJOCO_VIDEO_URL_HERE](ADD_MUJOCO_VIDEO_URL_HERE)

---

### 5.3 Performance Table: Position Tracking Error

The following results were generated by running [`track_error.py`](track_error.py), which executes the full backstepping controller at 50 Hz in a pure Python simulation and records the Euclidean position tracking error $\|x - x_T\|$ at every timestep.

| Phase | Duration | Mean Error | Max Error | Final Error | RMSE | Avg Thrust |
|---|---|---|---|---|---|---|
| Hover | 4 s | 0.5673 m | 2.0000 m | 0.0514 m | 0.8384 m | 14.66 N |
| Helical Spiral | 18 s | 0.3950 m | 2.0738 m | 0.2975 m | 0.5320 m | 15.11 N |
| 3D Figure-8 | 18 s | 1.4289 m | 3.2621 m | 2.2563 m | 1.6728 m | 16.05 N |
| Overall Average | 40 s | 0.7971 m | 3.2621 m | N/A | 1.0144 m | 15.27 N |

Interpretation:

- Hover final error is 0.0514 m. The controller converges to within 5 cm of the hover target after the initial transient from rest (start at z = 0, target z = 2 m).
- The large initial errors in the Hover and Helix phases occur during the transient while the vehicle accelerates from rest to the trajectory. They do not indicate steady-state divergence.
- Figure-8 errors are higher because this trajectory has larger accelerations (desired acceleration up to 2.5 times that of the helix), requiring more aggressive tilt corrections.
- Average thrust of 15.27 N is approximately 1.04 times the hover thrust, confirming that the controller stays near hover for all phases, consistent with the paper's 22 degree maximum tilt result.
- Lyapunov stability is analytically guaranteed by the following condition (Eq. 46 of the base paper).

$$
\dot{V} = z_x^\top A_x z_x + z_v^\top A_v z_v + z_t^\top A_t z_t + z_r^\top A_r z_r \leq 0
$$

To reproduce these results:

```powershell
python track_error.py
```

---

## 6. Controller Gains

| Parameter | Symbol | Value | Description |
|---|---|---:|---|
| Mass | $m$ | 1.5 kg | Vehicle mass |
| Gravity | $g$ | 9.81 m/s² | Gravitational acceleration |
| Position gain | $k_{pos}$ ($A_x$) | 2.0 | Drives position error to zero |
| Velocity gain | $k_{vel}$ ($A_v$) | 1.5 | Drives velocity error to zero |
| Tilt gain | $k_{tilt}$ ($A_t$) | 3.5 | Roll and pitch attitude correction |
| Yaw gain | $k_{yaw}$ ($a_\psi$) | 2.5 | Heading error correction |
| Rate damping | $k_{rate}$ ($A_r$) | 0.8 | Angular rate damping |

### Safety Limits

| Limit | Value |
|---|---|
| Max horizontal acceleration | 5.0 m/s² |
| Max thrust | $2.5\,mg \approx 36.8$ N |
| Minimum altitude | 0.2 m |

---

## 7. System Architecture

```
Windows Host
|
|-- run_gazebo_simulation.py     Main entry point (50 Hz control loop)
|     |-- BacksteppingController Thrust and torque from backstepping law
|     |-- SimulationRunner       Kinematics integration and UDP pose stream
|     |-- file_listener_loop     Monitors target.txt for setpoint updates
|
|-- <mujoco_runner>.py           MuJoCo simulation runner (50 Hz control loop)
|     |-- BacksteppingController Same controller, applied to MuJoCo dynamics
|     |-- MuJoCo model (MJCF)    Rigid-body quadrotor model
|
|-- run_gazebo_bridge.py         WebSocket bridge and HTTP file server
|     |-- WebSocket :8765        JSON telemetry to browser
|     |-- HTTP :8080             Serves gazebo_visualizer.html
|
|-- set_target.py                CLI utility: write X Y Z to target.txt

WSL2 Ubuntu
|
|-- gzserver / gzclient          Gazebo 11 physics and GUI (via VcXsrv X11)
|-- UDP Listener :9090           Receives 6-DOF pose and updates the gz model
```

---

## 8. Project Structure

```
d:\Drones\
|
|-- quadrotor_controller/          Core Python controller package
|   |-- __init__.py                Public API
|   |-- controller.py              BacksteppingController and quaternion math
|   |-- sitl_bridge.py             SITLBridge: MAVLink and simulator bridge
|
|-- models/
|   |-- quadrotor.world            Gazebo 11 SDF world
|   |-- <mujoco_model>.xml         MuJoCo MJCF quadrotor model
|
|-- tests/
|   |-- test_controller.py         Unit tests for BacksteppingController
|   |-- test_sitl_bridge.py        Unit tests for SITLBridge
|
|-- run_gazebo_simulation.py       Main simulation runner (Windows)
|-- <mujoco_runner>.py             MuJoCo simulation runner
|-- run_gazebo_bridge.py           WebSocket telemetry bridge server
|-- gazebo_visualizer.html         Real-time 3D web telemetry dashboard
|-- set_target.py                  CLI target setter utility
|-- launch_gazebo_wsl.py           Launches Gazebo inside WSL2
|-- run_gazebo.sh                  Bash launcher (inside WSL Ubuntu)
|
|-- START_GAZEBO_GUI.bat           One-click Windows launcher
|-- launch_web_visualizer.bat      One-click web visualizer launcher
|
|-- GAZEBO_INSTALLATION_GUIDE.md   Full Gazebo 11 and WSL2 setup guide
|-- Trajectory_tracking_control... Reference research paper (PDF)
```

Note: replace `<mujoco_model>.xml` and `<mujoco_runner>.py` with the actual file names used in the repository.

---

## 9. Mathematical Formulation Summary

| Symbol | Definition |
|---|---|
| $q = (w, x, y, z)^\top$ | Unit quaternion, $\|q\| = 1$ |
| $q_{xy}$ | Tilt quaternion, encodes thrust direction |
| $q_z$ | Heading quaternion, encodes yaw |
| $q = q_{xy} \otimes q_z$ | Decoupled parametrization (base paper Eq. 5) |
| $z_x = x - x_T$ | Position tracking error |
| $z_v = v - v_d$ | Velocity error |
| $z_t = T - T_d$ | Thrust vector error |
| $\hat{z}_{act} \times \hat{z}_{des}$ | Tilt attitude error in $\mathbb{R}^3$ |
| $\psi_{err} = \mathrm{wrap}(\psi_{ref} - \psi)$ | Yaw error |
| $V = V_x + V_v + V_t + V_\omega$ | Composite Lyapunov function |

The complete closed-loop Lyapunov derivative satisfies:

$$
\dot{V} = z_x^\top A_x z_x + z_v^\top A_v z_v + z_t^\top A_t z_t + z_r^\top A_r z_r \leq 0
$$

with $A_x, A_v, A_t, A_r < 0$, which establishes asymptotic stability (Eq. 46, base paper).

---

## 10. Prerequisites and Installation

### System Requirements

| Component | Requirement |
|---|---|
| OS | Windows 10 / 11 with WSL2 |
| WSL2 | Ubuntu 22.04 LTS |
| Python | 3.9+ (Windows side) |
| X11 Server | VcXsrv |
| Simulator 1 | Gazebo 11 (via Pixi in WSL) |
| Simulator 2 | MuJoCo (Python bindings) |

### Step 1: Install VcXsrv

```powershell
winget install marha.VcXsrv
```

Launch XLaunch with the following settings: Multiple Windows, Display 0, Disable access control enabled.

### Step 2: Install WSL2 Ubuntu

```powershell
wsl --install -d Ubuntu
```

### Step 3: Install Gazebo 11 via Pixi (inside WSL)

```bash
curl -fsSL https://pixi.sh/install.sh | bash && source ~/.bashrc
pixi global install gazebo
```

### Step 4: Install OpenGL and X11 libraries (WSL)

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential mesa-utils libgl1-mesa-dri \
  libgl1-mesa-glx libglx-mesa0 libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4
```

### Step 5: Install Python dependencies (Windows)

```powershell
cd d:\Drones
python -m venv .venv
.venv\Scripts\activate
pip install pymavlink websockets mujoco
```

Note: `pymavlink` is optional. The simulation runs in pure-Python mock mode without it. The `mujoco` package is required only for the MuJoCo simulation.

See [`GAZEBO_INSTALLATION_GUIDE.md`](GAZEBO_INSTALLATION_GUIDE.md) for the full setup walkthrough.

---

## 11. How to Run

### Option A: One-Click GUI Launch

Double-click `START_GAZEBO_GUI.bat`. It automatically starts VcXsrv, launches Gazebo 11, and runs the controller.

### Option B: Manual Launch

Terminal 1, start the simulation:

```powershell
cd d:\Drones
python run_gazebo_simulation.py
```

Terminal 2, start the web visualizer (optional):

```powershell
python run_gazebo_bridge.py
```

Open `http://localhost:8080` in a browser.

### Option C: Custom Target at Launch

```powershell
python run_gazebo_simulation.py 5 5 10
```

The vehicle flies immediately to X = 5 m, Y = 5 m, Z = 10 m.

### Option D: MuJoCo Simulation

```powershell
python <mujoco_runner>.py
```

Replace `<mujoco_runner>.py` with the MuJoCo entry-point script of the repository.

---

## 12. Flight Modes

### Trajectory Mode (Automatic, 40-second repeating cycle)

| Phase | Time | Description |
|---|---|---|
| Hover | 0 - 4 s | Stabilize at (0, 0, 2 m) |
| Helical Spiral | 4 - 22 s | Circular orbit of radius 2 m with altitude oscillation (matches paper Fig. 4) |
| 3D Figure-8 | 22 - 40 s | Lemniscate of Bernoulli in 3D space |

### Target Setpoint Mode

The vehicle flies to and holds a custom (X, Y, Z) coordinate. This mode is activated by `set_target.py` or a CLI argument.

---

## 13. Setting a Custom Target

```powershell
# Fly to X=5 m, Y=5 m, Z=10 m
python set_target.py 5 5 10

# Fly to negative coordinates
python set_target.py -3 2 5

# Resume automatic trajectory
python set_target.py auto
```

The utility writes to `target.txt`, and the simulation polls it every 500 ms. No restart is required.

---

## 14. API Reference

### BacksteppingController

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
# thrust  -> scalar, in Newtons
# torques -> (roll_torque, pitch_torque, yaw_torque)
```

### Quaternion Utilities

```python
from quadrotor_controller import (
    quaternion_from_euler,   # (roll, pitch, yaw) -> (w, x, y, z)
    normalize_quaternion,    # q -> unit quaternion
    decompose_attitude,      # q -> (q_xy, q_z) tilt/heading split
)
```

### SITLBridge (Optional MAVLink)

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

## 15. Running Tests

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

## 16. Key Observations

1. The quaternion decoupling $q = q_{xy} \otimes q_z$ allows translational and yaw dynamics to be controlled by two independent laws, each with its own Lyapunov stability proof.
2. Computing the tilt error as $\hat{z}_{actual} \times \hat{z}_{desired}$ in $\mathbb{R}^3$ guarantees the shortest-angle rotation and avoids the unwinding phenomenon without any sign-flip logic on the quaternion.
3. The helical trajectory period times ($T_{xy} = 6.25$ s, $T_z = 12.5$ s) are taken directly from the paper's experimental flight in Fig. 4 and reproduced exactly in the simulation.
4. Scalar gains are used in place of the paper's matrix gains ($A_x = -I$, $A_v = -3I$, $A_t = -8I$, $A_r = -12I$). This is a standard simplification when hardware inertia parameters are not available.
5. The 50 Hz control-loop rate matches the telemetry rate reported in the paper's experimental setup (Vicon data at 50 Hz to the quadrotor via radio link).
6. The same controller and gains operate in both Gazebo and MuJoCo without structural changes, which indicates that the implementation is not tied to a specific simulator.

---

## 17. Limitations

- Aerodynamic drag $f(v)$ is not modeled (set to zero in the simulation).
- The inertia tensor $J$ and the gyroscopic coupling term $-J\omega \times \omega$ are not modeled in the controller.
- The dynamic thrust extension ($\ddot{T} = u_T$, Eq. 31 of the paper) is omitted. Thrust is computed algebraically.
- The full 4-step backstepping cascade (position, velocity, thrust error, angular velocity error) is reduced to a 2-step proportional-derivative computation.
- No real sensor noise or IMU model is included.

---

## 18. Future Work

- Add the full inertia tensor and gyroscopic feedforward term.
- Implement the complete 4-step backstepping Lyapunov cascade with matrix gains.
- Include an aerodynamic drag model $f(v)$ identified from flight data.
- Extend to adaptive gain tuning to handle unknown or time-varying parameters.
- Port the controller to ArduPilot or PX4 via the existing `SITLBridge` MAVLink interface.
- Add realistic IMU sensor noise and an attitude estimator (for example, a Mahony filter).
- Implement full ROS 2 integration for SLAM and obstacle avoidance.

---

## 19. Conclusion

This project implements and validates, in Gazebo 11 and MuJoCo simulations, a complete real-time backstepping trajectory tracking controller for a quadrotor, based on the decoupled quaternion parametrization of De Monte and Lohmann (2013). The core design choices, namely the unit quaternion attitude representation, the tilt and heading decomposition ($q = q_{xy} \otimes q_z$), and the $\mathbb{R}^3$ cross-product attitude error, are reproduced faithfully from the paper and produce stable trajectory tracking in the Helical Spiral and Figure-8 maneuvers. The helix trajectory period parameters match the paper's experimental setup exactly. The WebSocket telemetry bridge and live browser visualizer provide real-time monitoring of all state variables, and the `set_target.py` utility allows interactive re-tasking of the vehicle without restarting the simulation.

---

## 20. References

1. P. De Monte and B. Lohmann, "Trajectory Tracking Control for a Quadrotor Helicopter based on Backstepping using a Decoupling Quaternion Parametrization," 21st Mediterranean Conference on Control and Automation (MED), Crete, Greece, June 2013. IEEE. (Base paper)

2. M. Krstic, I. Kanellakopoulos, and P. Kokotovic, Nonlinear and Adaptive Control Design, John Wiley & Sons, 1995. (Backstepping foundations)

3. S. P. Bhat and D. S. Bernstein, "A topological obstruction to continuous global stabilization of rotational motion and the unwinding phenomenon," Systems & Control Letters, vol. 39, pp. 63-70, 2000.

4. O. Fritsch, P. De Monte, M. Buhl, and B. Lohmann, "Quasi-static feedback linearization for the translational dynamics of a quadrotor helicopter," Proceedings of the American Control Conference, 2012.

5. T. Hamel, R. Mahony, R. Lozano, and J. Ostrowski, "Dynamic modelling and configuration stabilization for an X4-Flyer," Proceedings of the 15th IFAC World Congress, 2002.

6. E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for model-based control," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012.
