"""
Procedural city generator (config.yaml -> world).

Builds one list of primitives (boxes, cylinders, spheres) and writes it as
  * a MuJoCo include file   (paths.city_mujoco)  - also read by the PyBullet simulation
  * a Gazebo 11 SDF world   (paths.gazebo_world) - with the drone, target marker and pose plugin
so all three simulators fly in the same city.

Run:  python generate_world.py
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

from .config import cfg
from .units import PERCENT

Vec3 = Tuple[float, float, float]


@dataclass
class Prim:
    name: str
    kind: str                     # box | cylinder | sphere
    size: Tuple[float, ...]       # MuJoCo convention: box half extents, cylinder (radius, half height), sphere (radius,)
    pos: Vec3
    material: str
    yaw: float = 0.0              # rotation about z (rad)
    collide: bool = True


class CityBuilder:
    def __init__(self):
        self.w = cfg("world")
        self.rng = random.Random(self.w["seed"])
        self.prims: List[Prim] = []
        self._count: Dict[str, int] = {}
        self.extra_materials: Dict[str, dict] = {}
        self.pp = self.w["props"]          # prop proportions (config world.props)

    # ------------------------------------------------------------------ helpers
    def _name(self, base: str) -> str:
        n = self._count.get(base, 0)
        self._count[base] = n + 1
        return f"{base}_{n}"

    def box(self, base, pos, half, mat, yaw=0.0, collide=True):
        self.prims.append(Prim(self._name(base), "box", tuple(half), tuple(pos), mat, yaw, collide))

    def cyl(self, base, pos, radius, half_h, mat, collide=True):
        self.prims.append(Prim(self._name(base), "cylinder", (radius, half_h), tuple(pos), mat, 0.0, collide))

    def sphere(self, base, pos, radius, mat, collide=True):
        self.prims.append(Prim(self._name(base), "sphere", (radius,), tuple(pos), mat, 0.0, collide))

    # ------------------------------------------------------------------ build
    def build(self) -> List[Prim]:
        self.ground_surfaces()
        self.plaza()
        self.roads()
        self.sidewalks_and_street()
        self.buildings()
        self.cars()
        return self.prims

    def ground_surfaces(self):
        r = self.w["roads"]
        lw = r["line_width"]
        t = r["top"]
        P, R, A = self.w["plaza"]["half_size"], r["ring_outer"], r["avenue_half_width"]
        E = self.w["extent"]
        mid = (P + R) / 2.0
        half_ring = (R - P) / 2.0
        # ring road (4 sides) and avenues along x and y
        for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if sx:
                self.box("ring_road", (sx * mid, 0.0, t / 2), (half_ring, R, t / 2), "asphalt")
                self.box("avenue", (sx * (R + E) / 2, 0.0, t / 2), ((E - R) / 2, A, t / 2), "asphalt")
            else:
                self.box("ring_road", (0.0, sy * mid, t / 2), (P, half_ring, t / 2), "asphalt")
                self.box("avenue", (0.0, sy * (R + E) / 2, t / 2), (A, (E - R) / 2, t / 2), "asphalt")
        th = lw * self.pp["marking_thickness"]      # half thickness of the road paint
        z = t + th
        # avenue double yellow centre lines and white edge lines
        for axis in (0, 1):
            for sgn in (1, -1):
                c = sgn * (R + E) / 2
                L = (E - R) / 2
                for off in (-lw, lw):
                    p = (c, off, z) if axis == 0 else (off, c, z)
                    h = (L, lw / 2, th) if axis == 0 else (lw / 2, L, th)
                    self.box("line_yellow", p, h, "line_yellow", collide=False)
                for off in (-(A - 2 * lw), A - 2 * lw):
                    p = (c, off, z) if axis == 0 else (off, c, z)
                    h = (L, lw / 2, th) if axis == 0 else (lw / 2, L, th)
                    self.box("line_white", p, h, "line_white", collide=False)
        # ring road dashed centre line
        dash, gap = r["dash_length"], r["dash_gap"]
        n = int((2 * mid) // (dash + gap))
        for i in range(n):
            u = -mid + (i + 0.5) * (dash + gap)
            for p, h in (((u, mid, z), (dash / 2, lw / 2, th)), ((u, -mid, z), (dash / 2, lw / 2, th)),
                         ((mid, u, z), (lw / 2, dash / 2, th)), ((-mid, u, z), (lw / 2, dash / 2, th))):
                self.box("line_dash", p, h, "line_white", collide=False)
        # zebra crossings from the plaza across the ring road on every avenue
        st, cl = r["crosswalk_stripe"], r["crosswalk_length"]
        k = int((2 * A) // (2 * st))
        for i in range(k):
            u = -A + (i + 0.5) * 2 * st
            for sgn in (1, -1):
                self.box("zebra", (sgn * mid, u, z), (cl / 2, st / 2, th), "line_white", collide=False)
                self.box("zebra", (u, sgn * mid, z), (st / 2, cl / 2, th), "line_white", collide=False)

    def plaza(self):
        p = self.w["plaza"]
        P, top = p["half_size"], p["top"]
        self.box("plaza", (0.0, 0.0, top / 2), (P, P, top / 2), "paving")
        hr = p["helipad_radius"]
        hp = self.pp["helipad"]
        pt = hp["paint_half"]
        self.cyl("helipad", (0, 0, top + hp["pad_half"]), hr, hp["pad_half"], "helipad_dark", collide=False)
        self.cyl("helipad_ring", (0, 0, top + hp["ring_z"]), hr * hp["ring_fraction"], pt, "helipad_yellow", collide=False)
        self.cyl("helipad_inner", (0, 0, top + hp["inner_z"]), hr * hp["inner_fraction"], pt, "helipad_dark", collide=False)
        zh = top + hp["letter_z"]
        bx, (lx, ly), (cx, cy) = hp["letter_bar_x"], hp["letter_bar_half"], hp["letter_cross_half"]
        self.box("helipad_h", (-bx, 0, zh), (lx, ly, pt), "line_white", collide=False)
        self.box("helipad_h", (bx, 0, zh), (lx, ly, pt), "line_white", collide=False)
        self.box("helipad_h", (0, 0, zh), (cx, cy, pt), "line_white", collide=False)
        # avoidance-demo obstacles (fixed positions / sizes)
        ob = self.w["plaza_obstacles"]
        x, y = ob["clock_tower"]["pos"]
        hx, hy, hz = ob["clock_tower"]["half"]
        self.box("clock_tower", (x, y, top + hz), (hx, hy, hz), "sandstone")
        ct = self.pp["clock_tower"]
        rh, ov = ct["roof_half_height"], ct["roof_overhang"]
        self.box("clock_roof", (x, y, top + 2 * hz + rh), (hx + ov, hy + ov, rh), "roof")
        self.cyl("clock_spire", (x, y, top + 2 * hz + ct["spire_z"]), *ct["spire"], "metal")
        ft, fs = ct["face_thickness"], ct["face_half"]
        for ax, sg in ((0, 1), (0, -1), (1, 1), (1, -1)):
            off = (sg * (hx + ft), 0.0) if ax == 0 else (0.0, sg * (hy + ft))
            self.box("clock_face", (x + off[0], y + off[1], top + 2 * hz - ct["face_below_top"]),
                     (ft, fs, fs) if ax == 0 else (fs, ft, fs), "clock_face", collide=False)
        x, y = ob["cafe_pavilion"]["pos"]
        hx, hy, hz = ob["cafe_pavilion"]["half"]
        self.box("cafe", (x, y, top + hz), (hx, hy, hz), "concrete")
        cp = self.pp["cafe"]
        self.box("cafe_glass", (x, y, top + hz * cp["glass_z"]), (hx + cp["glass_out"], hy * cp["glass_width"], hz * cp["glass_height"]),
                 "glass_light", collide=False)
        ad = cp["awning_depth"]
        self.box("cafe_awning", (x + hx + ad, y, top + hz * cp["awning_z"]), (ad, hy, cp["awning_thickness"]), "awning", collide=False)
        rh, ov = cp["roof_half_height"], cp["roof_overhang"]
        self.box("cafe_roof", (x, y, top + 2 * hz + rh), (hx + ov, hy + ov, rh), "roof")
        x, y = ob["metro_entrance"]["pos"]
        hx, hy, hz = ob["metro_entrance"]["half"]
        self.box("metro", (x, y, top + hz), (hx, hy, hz), "dark_metal")
        mp = self.pp["metro"]
        gt, gf = mp["glass_thickness"], mp["glass_fraction"]
        self.box("metro_glass", (x, y + hy + gt, top + hz), (hx * gf, gt, hz * gf), "glass_light", collide=False)
        rh, ov = mp["roof_half_height"], mp["roof_overhang"]
        self.box("metro_roof", (x, y, top + 2 * hz + rh), (hx + ov, hy + ov, rh), "metal")
        for lx, ly in ob["lamps"]:
            self.street_lamp(lx, ly, top)
        # benches and planters on a ring away from the flight area
        b = p["benches_at"]
        for sx in (-1, 1):
            for sy in (-1, 1):
                pl = self.pp["planter"]
                ph, sh = pl["half_height"], pl["soil_half"]
                self.cyl("planter", (sx * b, sy * b, top + ph), pl["radius"], ph, "concrete")
                self.cyl("planter_soil", (sx * b, sy * b, top + 2 * ph + sh), pl["soil_radius"], sh, "soil", collide=False)
                self.tree(sx * b, sy * b, top + 2 * ph, scale=pl["tree_scale"])
        bo = self.pp["bench"]["offset"]
        for u in (-bo, bo):
            for sgn in (1, -1):
                self.bench(u, sgn * b, top, along_x=True)
                self.bench(sgn * b, u, top, along_x=False)

    def roads(self):
        pass  # surfaces and markings are created in ground_surfaces()

    def sidewalks_and_street(self):
        s, r = self.w["sidewalk"], self.w["roads"]
        R, A, E, W, h = r["ring_outer"], r["avenue_half_width"], self.w["extent"], s["width"], s["curb_height"]
        st = self.w["street"]
        # each quadrant: L-shaped sidewalk along the ring road and the two avenues
        for sx in (1, -1):
            for sy in (1, -1):
                # along the ring (outer edge), from avenue to avenue corner
                x0, x1 = A, R + W
                self.box("sidewalk", (sx * (x0 + x1) / 2, sy * (R + W / 2), h / 2), ((x1 - x0) / 2, W / 2, h / 2), "sidewalk")
                self.box("sidewalk", (sx * (R + W / 2), sy * (A + R) / 2, h / 2), (W / 2, (R - A) / 2, h / 2), "sidewalk")
                # along the avenues
                self.box("sidewalk", (sx * (R + W + E) / 2, sy * (A + W / 2), h / 2), ((E - R - W) / 2, W / 2, h / 2), "sidewalk")
                self.box("sidewalk", (sx * (A + W / 2), sy * (R + W + E) / 2, h / 2), (W / 2, (E - R - W) / 2, h / 2), "sidewalk")
                # trees and lamps along the avenue sidewalks
                u = R + W + st["tree_spacing"] / 2
                i = 0
                while u < E - 1.0:
                    if i % 2 == 0:
                        self.tree(sx * u, sy * (A + W / 2), h)
                        self.tree(sx * (A + W / 2), sy * u, h)
                    else:
                        lo = A + W * self.pp["lamp"]["sidewalk_fraction"]
                        self.street_lamp(sx * u, sy * lo, h)
                        self.street_lamp(sx * lo, sy * u, h)
                    u += st["tree_spacing"]
                    i += 1
                # corner trees on the ring sidewalk
                self.tree(sx * (R + W / 2), sy * (R + W / 2), h)
                if st["traffic_lights"]:
                    ko = self.pp["traffic_light"]["kerb_offset"]
                    self.traffic_light(sx * (A + ko), sy * (R + ko), h)

    # ------------------------------------------------------------------ props
    def tree(self, x, y, z0, scale=1.0):
        st = self.w["street"]
        r, H = st["tree_trunk"][0] * scale, st["tree_trunk"][1] * scale
        c1, c2 = st["tree_canopy"][0] * scale, st["tree_canopy"][1] * scale
        self.cyl("trunk", (x, y, z0 + H / 2), r, H / 2, "bark")
        tp = self.pp["tree"]
        mat = "leaves" if self.rng.random() < tp["light_leaves_probability"] else "leaves_dark"
        zc = z0 + H + c1 * tp["crown_raise"]
        self.sphere("canopy", (x, y, zc), c1, mat)
        # a few smaller offset clusters make the crown look less like a ball (visual only)
        for k in range(tp["clusters"]):
            a = self.rng.uniform(0, 2 * math.pi)
            off = c1 * tp["cluster_offset"]
            self.sphere("canopy", (x + off * math.cos(a), y + off * math.sin(a), zc + c1 * self.rng.uniform(*tp["cluster_lift"])),
                        c2, mat, collide=False)

    def street_lamp(self, x, y, z0):
        H = self.w["street"]["lamp_height"]
        lp = self.pp["lamp"]
        self.cyl("lamp_pole", (x, y, z0 + H / 2), lp["pole_radius"], H / 2, "dark_metal")
        self.box("lamp_arm", (x, y, z0 + H), lp["arm_half"], "dark_metal", collide=False)
        hx_, hz_ = lp["head_offset"]
        self.box("lamp_head", (x + hx_, y, z0 + H + hz_), lp["head_half"], "lamp_light", collide=False)

    def bench(self, x, y, z0, along_x=True):
        bp = self.pp["bench"]
        a_, b_ = bp["half"]
        hl = (a_, b_) if along_x else (b_, a_)
        self.box("bench_seat", (x, y, z0 + bp["seat_z"]), (hl[0], hl[1], bp["seat_half_thickness"]), "wood")
        leg = bp["leg_half"]
        for s in (-1, 1):
            off = (s * (hl[0] - bp["leg_inset"]), 0) if along_x else (0, s * (hl[1] - bp["leg_inset"]))
            self.box("bench_leg", (x + off[0], y + off[1], z0 + leg[2]), leg, "dark_metal", collide=False)

    def traffic_light(self, x, y, z0):
        tl = self.pp["traffic_light"]
        r_, hh = tl["pole"]
        self.cyl("signal_pole", (x, y, z0 + hh), r_, hh, "dark_metal")
        self.box("signal_box", (x, y, z0 + tl["box_z"]), tl["box_half"], "dark_metal", collide=False)
        fw, lr = tl["lamp_forward"], tl["lamp_radius"]
        self.sphere("signal_red", (x + fw, y, z0 + tl["red_z"]), lr, "traffic_red", collide=False)
        self.sphere("signal_green", (x + fw, y, z0 + tl["green_z"]), lr, "traffic_green", collide=False)

    # ------------------------------------------------------------------ buildings
    def buildings(self):
        b, r, s = self.w["blocks"], self.w["roads"], self.w["sidewalk"]
        pitch, n, lot = b["lot_pitch"], b["lots_per_axis"], b["lot_half_size"]
        clear = r["ring_outer"] + s["width"]
        side = r["avenue_half_width"] + s["width"]
        styles = list(self.w["facade_styles"].keys())
        for sx in (1, -1):
            for sy in (1, -1):
                for i in range(1, n + 1):
                    for j in range(1, n + 1):
                        cx, cy = i * pitch, j * pitch
                        if max(cx, cy) - lot < clear or min(cx, cy) - lot < side:
                            continue
                        far = max(i, j) == n
                        if far and self.rng.random() < b["tall_probability"]:
                            floors = self.rng.randint(*b["tall_floors"])
                            style = "tower"
                        else:
                            floors = self.rng.randint(*b["floors"])
                            style = self.rng.choice([s_ for s_ in styles if s_ != "tower"])
                        fx = lot * self.rng.uniform(*b["footprint_fraction"])
                        fy = lot * self.rng.uniform(*b["footprint_fraction"])
                        self.building(sx * cx, sy * cy, fx, fy, floors, style)

    def building(self, x, y, hx, hy, floors, style):
        fh = self.w["floor_height"]
        b = self.w["blocks"]
        wall, window = self.w["facade_styles"][style]
        z0 = self.w["sidewalk"]["curb_height"]
        H = floors * fh
        self.box("building", (x, y, z0 + H / 2), (hx, hy, H / 2), wall)
        # window bands, one per floor on every facade (visual only)
        bp = self.pp["building"]
        band = fh * (bp["window_band"] if style != "tower" else bp["tower_window_band"])
        d = bp["window_offset"]
        ww = bp["window_width"]
        for f in range(floors):
            zc = z0 + f * fh + fh * bp["window_z"]
            hb = band / 2
            if f == 0 and b["shopfront_floor"] and style != "tower":
                hb, zc = bp["shopfront_band"] * fh, z0 + fh * bp["shopfront_z"]
            for sgn in (1, -1):
                self.box("window", (x + sgn * (hx + d), y, zc), (d, hy * ww, hb), window, collide=False)
                self.box("window", (x, y + sgn * (hy + d), zc), (hx * ww, d, hb), window, collide=False)
        # roof: parapet slab, rooftop units, antenna on tall buildings
        rh, ov = bp["roof_half_height"], bp["roof_overhang"]
        self.box("roof", (x, y, z0 + H + rh), (hx + ov, hy + ov, rh), "roof")
        ua = bp["unit_area"]
        for _ in range(self.rng.randint(*b["rooftop_units"])):
            ux = x + self.rng.uniform(-hx * ua, hx * ua)
            uy = y + self.rng.uniform(-hy * ua, hy * ua)
            if style == "residential" and self.rng.random() < bp["water_tank_probability"]:
                self.cyl("water_tank", (ux, uy, z0 + H + bp["water_tank_z"]), *bp["water_tank"], "wood")
            else:
                self.box("ac_unit", (ux, uy, z0 + H + bp["ac_unit_z"]), bp["ac_unit_half"], "metal")
        if floors >= b["antenna_min_floors"]:
            self.cyl("antenna", (x, y, z0 + H + bp["antenna_z"]), *bp["antenna"], "metal")
            self.sphere("beacon", (x, y, z0 + H + bp["beacon_z"]), bp["beacon_radius"], "beacon", collide=False)

    # ------------------------------------------------------------------ cars
    def cars(self):
        st, r = self.w["street"], self.w["roads"]
        L, W, Hb = st["car_size"]
        A, R, E = r["avenue_half_width"], r["ring_outer"], self.w["extent"]
        colors = self.w["car_colors"]
        placed = 0
        tries = 0
        cp = self.pp["car"]
        while placed < st["cars"] and tries < st["cars"] * cp["tries_per_car"]:
            tries += 1
            along = self.rng.uniform(R + 2 * L, E - 2 * L)
            lane = self.rng.choice((-1, 1)) * (A - W - cp["kerb_gap"])       # parked at the kerb
            sgn = self.rng.choice((-1, 1))
            if self.rng.random() < 0.5:
                x, y, yaw = sgn * along, lane, 0.0
            else:
                x, y, yaw = lane, sgn * along, math.pi / 2
            if any(math.dist((x, y), (p.pos[0], p.pos[1])) < cp["min_spacing"] * L for p in self.prims if p.name.startswith("car_body")):
                continue
            color = colors[placed % len(colors)]
            self.car(x, y, yaw, color)
            placed += 1

    def car(self, x, y, yaw, rgba):
        L, W, Hb = self.w["street"]["car_size"]
        cp = self.pp["car"]
        mat = "car_" + "_".join(str(int(c * PERCENT)) for c in rgba[:3])
        self.extra_materials[mat] = {"rgba": rgba, "specular": cp["specular"], "shininess": cp["shininess"]}
        c, s = math.cos(yaw), math.sin(yaw)
        z_wheel = cp["wheel_radius"]
        half = (L, W, Hb) if yaw == 0.0 else (W, L, Hb)
        self.box("car_body", (x, y, z_wheel + Hb), half, mat)
        cab = tuple(k * f for k, f in zip((L, W, Hb), cp["cabin"]))
        cb = cp["cabin_back"]
        self.box("car_cabin", (x - c * L * cb, y - s * L * cb, z_wheel + 2 * Hb + cab[2]),
                 cab if yaw == 0.0 else (cab[1], cab[0], cab[2]), "glass_dark", collide=False)
        for a in (-1, 1):
            for b_ in (-1, 1):
                wx = x + c * a * L * cp["wheelbase"] - s * b_ * W
                wy = y + s * a * L * cp["wheelbase"] + c * b_ * W
                self.sphere("car_wheel", (wx, wy, z_wheel), z_wheel, "tyre", collide=False)

    def materials(self) -> Dict[str, dict]:
        mats = dict(self.w["materials"])
        mats.update(self.extra_materials)
        return mats


# ---------------------------------------------------------------------- writers

def _fmt(v: Sequence[float]) -> str:
    return " ".join(f"{x:.4g}" for x in v)


def write_mujoco(builder: CityBuilder, path: str) -> None:
    w = cfg("world")
    prims = builder.prims
    lines = ['<mujoco model="generated_city">',
             '  <!-- GENERATED by generate_world.py from config.yaml -> world. Do not edit by hand. -->',
             "  <asset>",
             f'    <texture name="sky" type="skybox" builtin="gradient" rgb1="{_fmt(w["sky"]["top"])}" '
             f'rgb2="{_fmt(w["sky"]["horizon"])}" width="512" height="512"/>']
    for name, m in builder.materials().items():
        attrs = f'rgba="{_fmt(m["rgba"])}"'
        for k in ("specular", "shininess", "reflectance"):
            if k in m:
                attrs += f' {k}="{m[k]}"'
        if m.get("texture") == "noise":
            lines.append(f'    <texture name="tex_{name}" type="2d" builtin="flat" mark="random" random="0.08" '
                         f'rgb1="{_fmt(m["rgba"][:3])}" rgb2="{_fmt(m["rgb2"])}" markrgb="{_fmt(m["rgb2"])}" width="512" height="512"/>')
            rep = w["ground_size"] / w["props"]["grass_tiles"] if name == "grass" else w["props"]["texture_repeat"]
            lines.append(f'    <material name="{name}" texture="tex_{name}" texrepeat="{rep:g} {rep:g}" {attrs}/>')
        else:
            lines.append(f'    <material name="{name}" {attrs}/>')
    lines += ["  </asset>", "  <worldbody>",
              f'    <geom name="ground" type="plane" size="{w["ground_size"]:g} {w["ground_size"]:g} 0.1" material="grass"/>',
              f'    <light name="sun" directional="true" pos="0 0 60" dir="{_fmt(w["sun"]["direction"])}" '
              f'diffuse="{_fmt(w["sun"]["diffuse"])}" castshadow="true"/>']
    for p in prims:
        size = _fmt(p.size)
        extra = "" if p.collide else ' contype="0" conaffinity="0"'
        rot = f' euler="0 0 {math.degrees(p.yaw):.4g}"' if p.yaw else ""
        lines.append(f'    <geom name="{p.name}" type="{p.kind}" size="{size}" pos="{_fmt(p.pos)}"{rot} material="{p.material}"{extra}/>')
    lines += ["  </worldbody>", "</mujoco>", ""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def _sdf_geometry(p: Prim) -> str:
    if p.kind == "box":
        return f"<box><size>{_fmt([2 * s for s in p.size])}</size></box>"
    if p.kind == "cylinder":
        return f"<cylinder><radius>{p.size[0]:.4g}</radius><length>{2 * p.size[1]:.4g}</length></cylinder>"
    return f"<sphere><radius>{p.size[0]:.4g}</radius></sphere>"


def write_gazebo(builder: CityBuilder, path: str, drone_template: str, plugin: dict) -> None:
    w = cfg("world")
    prims = builder.prims
    mats = builder.materials()
    sun = w["sun"]
    out = ['<?xml version="1.0" ?>',
           '<!-- GENERATED by generate_world.py from config.yaml -> world. Do not edit by hand. -->',
           '<sdf version="1.6">', '  <world name="city">',
           f'    <scene><ambient>{_fmt(sun["ambient"])} 1</ambient><background>{_fmt(w["sky"]["horizon"])} 1</background>'
           '<shadows>true</shadows><sky><clouds><speed>4</speed></clouds></sky></scene>',
           '    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 60 0 0 0</pose>'
           f'<diffuse>{_fmt(sun["diffuse"])} 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
           f'<direction>{_fmt(sun["direction"])}</direction></light>',
           '    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>',
           '    <gui fullscreen="0"><camera name="user_camera">'
           f'<pose>{_fmt(plugin["camera_pose"])}</pose></camera></gui>',
           '    <model name="ground"><static>true</static><link name="link">'
           f'<collision name="c"><geometry><plane><normal>0 0 1</normal><size>{w["ground_size"] * 2:g} {w["ground_size"] * 2:g}</size></plane></geometry></collision>'
           f'<visual name="v"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>{w["ground_size"] * 2:g} {w["ground_size"] * 2:g}</size></plane></geometry>'
           f'<material><ambient>{_fmt(mats["grass"]["rgba"])}</ambient><diffuse>{_fmt(mats["grass"]["rgba"])}</diffuse></material></visual>'
           '</link></model>',
           '    <model name="city"><static>true</static><link name="link">']
    for p in prims:
        m = mats[p.material]
        pose = f"<pose>{_fmt(p.pos)} 0 0 {p.yaw:.4g}</pose>"
        geom = _sdf_geometry(p)
        spec = m.get("specular", w["props"]["default_specular"])
        out.append(f'      <visual name="{p.name}">{pose}<geometry>{geom}</geometry><material>'
                   f'<ambient>{_fmt(m["rgba"])}</ambient><diffuse>{_fmt(m["rgba"])}</diffuse>'
                   f'<specular>{spec:g} {spec:g} {spec:g} 1</specular></material></visual>')
        if p.collide:
            out.append(f'      <collision name="{p.name}_c">{pose}<geometry>{geom}</geometry></collision>')
    out.append("    </link></model>")
    with open(drone_template, encoding="utf-8") as f:
        drone = f.read()
    out.append(drone.replace("<!--POSE_PLUGIN-->", plugin["drone"]))
    out.append(plugin["target_model"])
    out += ["  </world>", "</sdf>", ""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out))
