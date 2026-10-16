"""
Static world geometry read from the MuJoCo model (paths.mujoco_model).

The city layout is defined once, in models/quadrotor.xml; the PyBullet simulation
rebuilds the same buildings, lamps, trees and markings from it, so the two
simulators always share one world.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .config import cfg, project_path


@dataclass
class StaticGeom:
    name: str
    kind: str                       # box | cylinder | sphere | capsule | ellipsoid
    size: Tuple[float, ...]         # MuJoCo size (half extents / radius, half length)
    pos: Tuple[float, float, float]
    quat_xyzw: Tuple[float, float, float, float]
    rgba: Tuple[float, float, float, float]
    collide: bool


def load_static_geoms(model_path: Optional[str] = None, skip_bodies: Sequence[str] = ()) -> List[StaticGeom]:
    """All geoms fixed to the world (not on free/mocap bodies), except planes."""
    import mujoco

    m = mujoco.MjModel.from_xml_path(model_path or project_path("mujoco_model"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    kinds = {mujoco.mjtGeom.mjGEOM_BOX: "box", mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
             mujoco.mjtGeom.mjGEOM_SPHERE: "sphere", mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
             mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid"}
    skip = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in skip_bodies}
    min_half = cfg("numerics.min_shape_half_size")
    out = []
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        if b in skip or _moving(m, b):
            continue
        kind = kinds.get(int(m.geom_type[g]))
        if kind is None:
            continue
        mat = int(m.geom_matid[g])
        rgba = tuple(float(c) for c in (m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]))
        qq = np.zeros(4)
        mujoco.mju_mat2Quat(qq, d.geom_xmat[g])
        size = tuple(max(float(s), min_half) for s in m.geom_size[g])
        out.append(StaticGeom(
            name=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}",
            kind=kind, size=size, pos=tuple(float(x) for x in d.geom_xpos[g]),
            quat_xyzw=(float(qq[1]), float(qq[2]), float(qq[3]), float(qq[0])), rgba=rgba,
            collide=bool(m.geom_contype[g] or m.geom_conaffinity[g])))
    return out


def _moving(m, body) -> bool:
    """True if the body or one of its ancestors has a joint (is not part of the static world)."""
    while body > 0:
        if m.body_jntnum[body] > 0 or m.body_mocapid[body] >= 0:
            return True
        body = m.body_parentid[body]
    return False
