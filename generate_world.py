"""
Generates the city for all simulators from config.yaml -> world:
  * paths.city_mujoco   (MuJoCo include file, also used by PyBullet)
  * paths.gazebo_world  (Gazebo 11 world with the city, drone, target marker and pose plugin)

Usage:  python generate_world.py            (re-run after changing the world section)
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, cfg, project_path  # noqa: E402
from quadrotor_controller.world_builder import CityBuilder, write_gazebo, write_mujoco  # noqa: E402


def gazebo_plugin_xml():
    lib = cfg("gazebo.plugin_library")

    def plugin(port):
        return f'<plugin name="udp_pose" filename="{lib}"><port>{port}</port></plugin>'

    target = ('    <model name="target_marker"><pose>0 0 1 0 0 0</pose><link name="link"><kinematic>true</kinematic>'
              '<gravity>false</gravity><visual name="sphere"><geometry><sphere><radius>0.12</radius></sphere></geometry>'
              '<material><ambient>1 0.2 0.3 0.6</ambient><diffuse>1 0.2 0.3 0.6</diffuse><emissive>0.6 0.1 0.15 1</emissive>'
              '</material><transparency>0.3</transparency></visual></link>'
              f'{plugin(cfg("gazebo.target_udp_port"))}</model>')
    return {"drone": plugin(cfg("gazebo.udp_port")), "target_model": target, "camera_pose": cfg("gazebo.camera_pose")}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    add_config_arguments(ap)
    apply_config_arguments(ap.parse_args())
    b = CityBuilder()
    prims = b.build()
    write_mujoco(b, project_path("city_mujoco"))
    write_gazebo(b, project_path("gazebo_world"), project_path("gazebo_drone_template"), gazebo_plugin_xml())
    n_col = sum(p.collide for p in prims)
    print(f"[world] {len(prims)} primitives ({n_col} with collision)")
    print(f"[world] MuJoCo / PyBullet : {cfg('paths.city_mujoco')}")
    print(f"[world] Gazebo            : {cfg('paths.gazebo_world')}")


if __name__ == "__main__":
    main()
