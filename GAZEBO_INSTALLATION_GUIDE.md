# 🚁 Native Gazebo 11 Multi-Robot Simulator Installation Guide (WSL Ubuntu + Windows)

This document provides the step-by-step instructions to set up **Native Gazebo 11** inside **WSL2 Ubuntu** with **VcXsrv 3D Desktop Acceleration** on Windows.

---

## 📋 Prerequisites & Overview Architecture

* **Operating System:** Windows 10 / 11 with WSL2 enabled.
* **Linux Subsystem:** Ubuntu 22.04 LTS (or Ubuntu 20.04 LTS).
* **X11 Display Server:** VcXsrv X-Server for Windows.
* **Gazebo Version:** Gazebo 11.15.1 (installed via Pixi/Conda package environment).

---

## 🛠️ Step-by-Step Installation Process

### Step 1: Install VcXsrv X-Server on Windows Host

1. Open PowerShell on Windows as Administrator and install VcXsrv via Winget:
   ```powershell
   winget install marha.VcXsrv
   ```
   *(Or download installer directly from [SourceForge VcXsrv](https://sourceforge.net/projects/vcxsrv/)).*

2. **Configure VcXsrv Settings:**
   * Launch **XLaunch** from the Windows Start menu.
   * Select **Multiple windows** -> Display number: `0` -> Click Next.
   * Select **Start no client** -> Click Next.
   * **IMPORTANT:** Check **Disable access control** (allows WSL X11 graphics connection).
   * Click **Finish**.

---

### Step 2: Install WSL2 & Ubuntu Packages

1. Open PowerShell as Administrator and ensure WSL2 Ubuntu is installed:
   ```powershell
   wsl --install -d Ubuntu
   ```

2. Open WSL Ubuntu terminal and update system repositories:
   ```bash
   sudo apt-get update && sudo apt-get upgrade -y
   ```

3. Install required OpenGL rendering, X11 libraries, and build tools:
   ```bash
   sudo apt-get install -y \
     build-essential \
     g++ \
     git \
     curl \
     psmisc \
     mesa-utils \
     libgl1-mesa-dri \
     libgl1-mesa-glx \
     libglx-mesa0 \
     libxcb-cursor0 \
     libxcb-xinerama0 \
     libxcb-icccm4
   ```

---

### Step 3: Install Native Gazebo 11 via Pixi Package Manager

Using Pixi guarantees clean dependency isolation for Gazebo 11 binaries, OGRE 3D rendering plugins, and ROS/Ignition math libraries.

1. **Install Pixi in WSL Ubuntu:**
   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   source ~/.bashrc
   ```

2. **Create Gazebo 11 Environment & Install Binaries:**
   ```bash
   pixi global install gazebo
   ```
   *(Or create dedicated Pixi environment):*
   ```bash
   pixi init /root/.pixi/envs/gazebo
   pixi add --manifest-path /root/.pixi/envs/gazebo/pixi.toml gazebo
   ```

---

### Step 4: Configure WSL Display & OpenGL Environment Variables

Add the following environment variables to your WSL `.bashrc` or launch script:

```bash
# Set X11 DISPLAY routing to Windows Host VcXsrv Server
export DISPLAY=$(ip route show default | awk '{print $3; exit}'):0

# Set Software OpenGL Rendering mode for WSL compatibility
export LIBGL_ALWAYS_SOFTWARE=1

# Activate Gazebo OGRE Resource Paths
export CONDA_PREFIX="/root/.pixi/envs/gazebo"
export PATH="/root/.pixi/envs/gazebo/bin:$PATH"
export LD_LIBRARY_PATH="/root/.pixi/envs/gazebo/lib"
source /root/.pixi/envs/gazebo/etc/conda/activate.d/gazebo_activate.sh
```

---

### Step 5: Launching & Verifying Native Gazebo 11

1. **Clean up any stale processes:**
   ```bash
   killall -9 gzserver gzclient gazebo 2>/dev/null || true
   ```

2. **Launch Gazebo 3D World:**
   ```bash
   gazebo --verbose /mnt/d/Drones/models/quadrotor.world
   ```

3. **1-Click Windows Shortcut Launch:**
   * Double-click `d:\Drones\launch_gazebo.bat` on Windows.
   * Automatically starts VcXsrv, sets environment variables, and launches Native Gazebo 11!

---

## 🎮 Python Trajectory Controller Integration

Run the Backstepping Quaternion Controller script from Windows or WSL:

```powershell
python d:\Drones\run_gazebo_simulation.py
```

* **Position Synchronization:** Sends real-time 6-DOF model pose updates directly to Gazebo Master (`/root/.pixi/envs/gazebo/bin/gz model -m quadrotor -x ...`).
