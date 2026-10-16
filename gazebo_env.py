"""
Prints 'export NAME=value' lines with the Gazebo settings from config.yaml, for run_gazebo.sh:

    eval "$(python3 gazebo_env.py)"

Uses only the standard library (loads quadrotor_controller/config.py directly), so it
runs with the plain Ubuntu python3 inside WSL.
"""
import importlib.util
import os
import shlex

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("drones_config", os.path.join(HERE, "quadrotor_controller", "config.py"))
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
config.PROJECT_ROOT = HERE
cfg = config.cfg

values = {
    "DRONES_DIR": HERE,
    "PIXI_ENV": cfg("gazebo.pixi_env"),
    "GAZEBO_MASTER_URI": cfg("gazebo.master_uri"),
    "GAZEBO_IP": cfg("gazebo.gazebo_ip"),
    "GAZEBO_WORLD": os.path.join(HERE, cfg("paths.gazebo_world")),
    "EXTRA_LD_LIBRARY_PATH": cfg("gazebo.extra_ld_library_path"),
    "SOFTWARE_RENDERING": "1" if cfg("gazebo.software_rendering") else "0",
    "CLEANUP_WAIT": str(cfg("gazebo.cleanup_wait")),
    "PLUGIN_DIR": os.path.join(HERE, cfg("paths.gazebo_plugin_dir")),
    "PLUGIN_LIBRARY": cfg("gazebo.plugin_library"),
    "POSE_TRANSPORT": cfg("gazebo.pose_transport"),
}
print("\n".join(f"export {k}={shlex.quote(str(v))}" for k, v in values.items()))
