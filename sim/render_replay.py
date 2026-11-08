"""Render a captured trial (sim/motion_capture.py) offline as a high-quality video: 1920x1080, 30 fps, shadows,
multisampling, studio lighting, a slow camera drift and a lower-third caption.

Runs where MuJoCo's offscreen renderer works (the HF Jobs box, MUJOCO_GL=osmesa or egl), never during a trial:

    python sim/render_replay.py --lab tower --motion <session>/motion.jsonl --trace <trial>/trace.jsonl \
        --out /work/render/tower --title "Ordered three-cube tower" --program tower_order_v24.py \
        --score "5/5 dev, 9/10 held-out" [--stills 10,50,90] [--frames-only] [--workers 12]

The model is the lab's own: the same bench MJCF (sim/stack_bench.xml or sim/cable_bench.xml) with the dimos Metal
attached at the origin the way MujocoSimModule composes it. Only looks change, in memory (the lab's physics XML is
untouched): materials, lights, skybox, bench texture, render buffer size. Every frame sets qpos from the capture
(arm joints, the mimicking finger, the tracked free bodies) and calls mj_forward; nothing is simulated. Body poses
arrive at the sim's camera rate (5 Hz), so a body carried in the jaw is interpolated in the jaw's frame, which keeps
it locked to the fingers between samples; a free body is interpolated in the world frame.
Writes <out>/frames/%06d.jpg, then <out>/<name>.mp4 (H.264 yuv420p, CRF 18, faststart) unless --frames-only.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DIMOS = Path(os.environ.get("DIMOS_ROOT", Path.home() / "projects" / "dimos"))
LABS = {
    "tower": HERE / "stack_bench.xml",
    "cable": HERE / "cable_bench.xml",
    "cycle": HERE / "cable_bench.xml",
}
#: Camera presets per lab: lookat, distance, azimuth, elevation (MuJoCo free camera convention, degrees).
HERO = {
    "tower": dict(lookat=(0.17, 0.02, 0.17), distance=0.98, azimuth=140.0, elevation=-14.0),
    "cable": dict(lookat=(0.18, 0.0, 0.15), distance=0.95, azimuth=140.0, elevation=-17.0),
    "cycle": dict(lookat=(0.18, 0.0, 0.15), distance=0.95, azimuth=140.0, elevation=-17.0),
}
CLOSE = {
    "tower": dict(lookat=(0.20, 0.0, 0.08), distance=0.36, azimuth=205.0, elevation=-20.0),
    "cable": dict(lookat=(0.25, 0.0, 0.05), distance=0.30, azimuth=215.0, elevation=-28.0, follow="hub"),
    "cycle": dict(lookat=(0.25, 0.0, 0.05), distance=0.30, azimuth=215.0, elevation=-28.0, follow="hub"),
}

# ---------------------------------------------------------------- the scene, prettified in memory


def _set(el: ET.Element, **attrs) -> ET.Element:
    for k, v in attrs.items():
        el.set(k, v)
    return el


def render_scene_xml(lab_xml: Path, width: int, height: int) -> str:
    """The lab's bench MJCF with render-only looks: same bodies, geoms and sizes."""
    root = ET.parse(lab_xml).getroot()
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    for child in list(visual):
        visual.remove(child)
    _set(ET.SubElement(visual, "global"), offwidth=str(width), offheight=str(height), azimuth="140", elevation="-15")
    _set(ET.SubElement(visual, "quality"), shadowsize="4096", offsamples="8")
    _set(ET.SubElement(visual, "headlight"), ambient="0.18 0.18 0.2", diffuse="0.22 0.22 0.22", specular="0 0 0")
    _set(ET.SubElement(visual, "map"), znear="0.01", zfar="20", shadowclip="1.6", shadowscale="0.6", haze="0.6")
    _set(ET.SubElement(visual, "rgba"), haze="0.80 0.83 0.88 1")

    asset = root.find("asset")
    for tex in asset.findall("texture"):
        if tex.get("type") == "skybox":
            _set(tex, builtin="gradient", rgb1="0.93 0.94 0.96", rgb2="0.52 0.56 0.62", width="800", height="4800")
        if tex.get("name") == "groundplane":
            _set(tex, builtin="flat", rgb1="0.36 0.38 0.41", rgb2="0.36 0.38 0.41", mark="none")
    _set(ET.SubElement(asset, "texture"), name="bench_grid", type="2d", builtin="checker", mark="edge",
         rgb1="0.66 0.67 0.68", rgb2="0.66 0.67 0.68", markrgb="0.55 0.56 0.58", width="512", height="512")
    looks = {
        "groundplane": dict(specular="0.05", shininess="0.1", reflectance="0"),
        "bench_top": dict(texture="bench_grid", texrepeat="20 20", texuniform="true", rgba="1 1 1 1",
                          specular="0.15", shininess="0.25", reflectance="0"),
        "bench_leg": dict(rgba="0.22 0.23 0.25 1", specular="0.4", shininess="0.5"),
        # foam cubes and sticks: saturated, matte
        "red": dict(rgba="0.86 0.08 0.10 1", specular="0.08", shininess="0.12"),
        "blue": dict(rgba="0.06 0.36 0.86 1", specular="0.08", shininess="0.12"),
        "yellow": dict(rgba="1.0 0.80 0.05 1", specular="0.08", shininess="0.12"),
        # cable set: satin black plastic hub, glossy plug housings, steel blades
        "hub_black": dict(rgba="0.24 0.25 0.27 1", specular="0.45", shininess="0.6"),
        "blade_steel": dict(rgba="0.80 0.81 0.84 1", specular="0.9", shininess="0.9"),
        "plug_orange": dict(rgba="1.0 0.42 0.04 1", specular="0.3", shininess="0.5"),
        "plug_green": dict(rgba="0.08 0.78 0.30 1", specular="0.3", shininess="0.5"),
        "plug_magenta": dict(rgba="0.88 0.10 0.68 1", specular="0.3", shininess="0.5"),
        "cable_dark": dict(rgba="0.15 0.15 0.16 1", specular="0.2", shininess="0.3"),
    }
    for mat in asset.findall("material"):
        if mat.get("name") in looks:
            _set(mat, **looks[mat.get("name")])
    # the arm: dark anodized links, brushed aluminium jaw mount, steel-grey fingers
    _set(ET.SubElement(asset, "material"), name="arm_anodized", rgba="0.17 0.18 0.20 1", specular="0.55", shininess="0.7")
    _set(ET.SubElement(asset, "material"), name="arm_base", rgba="0.12 0.12 0.13 1", specular="0.4", shininess="0.6")
    _set(ET.SubElement(asset, "material"), name="arm_mount", rgba="0.72 0.74 0.77 1", specular="0.7", shininess="0.8")
    _set(ET.SubElement(asset, "material"), name="arm_finger", rgba="0.45 0.47 0.50 1", specular="0.6", shininess="0.7")

    world = root.find("worldbody")
    for light in world.findall("light"):
        world.remove(light)
    # key: high front-right, shadows; fill: left, soft; rim: behind the arm
    lights = [
        dict(name="key", type="directional", castshadow="true", pos="1.2 -1.0 1.6", dir="-0.55 0.45 -0.7",
             diffuse="0.72 0.70 0.66", specular="0.35 0.35 0.35", ambient="0.06 0.06 0.07"),
        dict(name="fill", type="directional", castshadow="false", pos="0.4 1.2 0.8", dir="-0.1 -0.8 -0.55",
             diffuse="0.26 0.28 0.32", specular="0.05 0.05 0.05"),
        dict(name="rim", type="directional", castshadow="false", pos="-0.8 0.2 0.9", dir="0.75 -0.1 -0.45",
             diffuse="0.30 0.30 0.32", specular="0.4 0.4 0.4"),
    ]
    for i, attrs in enumerate(lights):
        world.insert(i, _set(ET.Element("light"), **attrs))
    for geom in world.iter("geom"):
        if geom.get("name") == "floor":
            geom.set("size", "4 4 0.05")
    return ET.tostring(root, encoding="unicode")


def build_model(lab: str, width: int, height: int):
    import mujoco

    sys.path.insert(0, str(DIMOS))
    from dimos.robot.manipulators.makermods_metal.config import METAL_MESHDIR, METAL_MJCF_PATH

    xml = render_scene_xml(LABS[lab], width, height)
    scene = mujoco.MjSpec.from_string(xml)
    robot = mujoco.MjSpec.from_file(str(METAL_MJCF_PATH))
    robot.meshdir = str(METAL_MESHDIR)
    scene.option.timestep = robot.option.timestep
    frame = scene.worldbody.add_frame(pos=[0.0, 0.0, 0.0])
    scene.attach(robot, prefix=None, frame=frame)  # MujocoSimModule: robot_id "" -> no prefix
    mats = {"base_link": "arm_base", "gripper_base": "arm_mount", "link7": "arm_finger", "link8": "arm_finger"}
    colors = {
        "arm_anodized": [0.17, 0.18, 0.20, 1], "arm_base": [0.12, 0.12, 0.13, 1],
        "arm_mount": [0.72, 0.74, 0.77, 1], "arm_finger": [0.45, 0.47, 0.50, 1],
    }
    for geom in scene.geoms:
        mesh = getattr(geom, "meshname", "")
        if geom.type == mujoco.mjtGeom.mjGEOM_MESH and mesh:
            mat = mats.get(mesh, "arm_anodized")
            geom.material = mat
            geom.rgba = colors[mat]
    model = scene.compile()
    return model


# ---------------------------------------------------------------- the capture


def load_capture(path: Path, t0: float, t1: float):
    jt, jq, names = [], [], None
    bodies: dict[str, list[tuple[float, list[float]]]] = {}
    with open(path) as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = row["t"]
            if t < t0 - 2.0 or t > t1 + 2.0:
                continue
            if row["k"] == "j":
                if names is None:
                    names = row["n"]
                if row["n"] != names:
                    continue
                jt.append(t)
                jq.append(row["q"])
            elif row["k"] == "b":
                for name, pose in row["b"].items():
                    bodies.setdefault(name, []).append((t, pose))
    if not jt:
        raise SystemExit(f"no joint rows in {path} between {t0} and {t1}")
    return np.array(jt), np.array(jq), [n.split("/")[-1] for n in names], bodies


def trial_window(trace: Path, pre: float, post: float) -> tuple[float, float, float]:
    ts = [json.loads(line)["ts"] for line in open(trace) if line.strip()]
    return ts[0] - pre, ts[-1] + post, ts[-1]


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _rotate(q, v):
    return _quat_mul(_quat_mul(q, np.array([0.0, *v])), _quat_conj(q))[1:]


def _slerp(a, b, s):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        q = a + s * (b - a)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - s) * th) * a + math.sin(s * th) * b) / math.sin(th)


class Replay:
    """qpos for any wall time inside the capture."""

    def __init__(self, model, jt, jq, jnames, bodies):
        import mujoco

        self.m = model
        self.d = mujoco.MjData(model)
        self.jt, self.jq = jt, jq
        # attached without a prefix, the arm's names may come out as "/joint1": match on the last path part
        joints = {model.joint(j).name.split("/")[-1]: j for j in range(model.njnt) if model.joint(j).name}
        links = {model.body(b).name.split("/")[-1]: b for b in range(model.nbody) if model.body(b).name}
        self.jaddr = [(i, model.jnt_qposadr[joints[n]]) for i, n in enumerate(jnames) if n in joints]
        self.j7 = model.jnt_qposadr[joints["joint7"]] if "joint7" in joints else None
        self.j8 = model.jnt_qposadr[joints["joint8"]] if "joint8" in joints else None
        self.tool = links["gripper_base"]
        self.bodies = {}
        for name, samples in bodies.items():
            bid = links.get(name, -1)
            if bid < 0 or model.body_jntnum[bid] != 1:
                continue
            adr = model.jnt_qposadr[model.body_jntadr[bid]]
            t = np.array([s[0] for s in samples])
            p = np.array([s[1] for s in samples], float)
            # the tool's pose at each sample, for in-jaw interpolation
            tool = [self._tool_pose(ti) for ti in t]
            rel = []
            for (tp, tq), pose in zip(tool, p):
                inv = _quat_conj(tq)
                rel.append(np.concatenate([_rotate(inv, pose[:3] - tp), _quat_mul(inv, pose[3:])]))
            self.bodies[name] = (adr, t, p, np.array(rel))
        self.bodies_missing = sorted(set(bodies) - set(self.bodies))

    def _set_joints(self, t):
        import mujoco

        for i, adr in self.jaddr:
            self.d.qpos[adr] = np.interp(t, self.jt, self.jq[:, i])
        if self.j7 is not None and self.j8 is not None:
            self.d.qpos[self.j8] = -self.d.qpos[self.j7]  # metal.xml: joint8 mimics joint7 with polycoef 0 -1
        mujoco.mj_kinematics(self.m, self.d)

    def _tool_pose(self, t):
        self._set_joints(t)
        return self.d.xpos[self.tool].copy(), self.d.xquat[self.tool].copy()

    def set(self, t):
        import mujoco

        self._set_joints(t)
        tp, tq = self.d.xpos[self.tool].copy(), self.d.xquat[self.tool].copy()
        for adr, ts, p, rel in self.bodies.values():
            i = int(np.searchsorted(ts, t))
            if i <= 0:
                pose = p[0]
            elif i >= len(ts):
                pose = p[-1]
            else:
                s = (t - ts[i - 1]) / max(ts[i] - ts[i - 1], 1e-6)
                a, b = p[i - 1], p[i]
                ra, rb = rel[i - 1], rel[i]
                moving = np.linalg.norm(b[:3] - a[:3]) > 0.002
                in_jaw = np.linalg.norm(rb[:3] - ra[:3]) < 0.004 and np.linalg.norm(ra[:3]) < 0.16
                if moving and in_jaw:
                    rpos = ra[:3] + s * (rb[:3] - ra[:3])
                    rq = _slerp(ra[3:], rb[3:], s)
                    pose = np.concatenate([tp + _rotate(tq, rpos), _quat_mul(tq, rq)])
                else:
                    pose = np.concatenate([a[:3] + s * (b[:3] - a[:3]), _slerp(a[3:], b[3:], s)])
            self.d.qpos[adr:adr + 3] = pose[:3]
            q = np.asarray(pose[3:], float)
            self.d.qpos[adr + 3:adr + 7] = q / np.linalg.norm(q)
        mujoco.mj_forward(self.m, self.d)


# ---------------------------------------------------------------- frames


def _camera(preset, t_rel, duration, drift):
    import mujoco

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = preset["lookat"]
    cam.distance = preset["distance"]
    phase = 0.0 if duration <= 0 else (t_rel / duration)
    cam.azimuth = preset["azimuth"] + drift * math.sin(math.pi * (phase - 0.5))  # one slow sweep over the video
    cam.elevation = preset["elevation"]
    return cam


def _font(size, bold=False):
    from PIL import ImageFont

    for path in (f"/usr/share/fonts/truetype/dejavu/DejaVuSans{'-Bold' if bold else ''}.ttf",
                 "/System/Library/Fonts/Helvetica.ttc"):
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def overlay(img, args, t_rel, t_prog_end_rel, height):
    from PIL import Image, ImageDraw

    s = height / 1080
    base = img.convert("RGBA")
    layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    x0, y1 = int(48 * s), base.size[1] - int(48 * s)
    lines = [(args.title, _font(int(40 * s), True), (255, 255, 255, 255)),
             (f"program {args.program}", _font(int(26 * s)), (215, 222, 232, 255)),
             (f"simulation  |  dimos Autoresearch  |  scored {args.score}", _font(int(24 * s)), (175, 186, 200, 255))]
    heights = [int(52 * s), int(36 * s), int(34 * s)]
    box_h = sum(heights) + int(36 * s)
    box_w = int(max(draw.textlength(t, font=f) for t, f, _ in lines) + 64 * s)
    y0 = y1 - box_h
    draw.rounded_rectangle([x0, y0, x0 + box_w, y1], radius=int(14 * s), fill=(14, 18, 24, 178))
    draw.rectangle([x0, y0 + int(14 * s), x0 + int(6 * s), y1 - int(14 * s)], fill=(64, 160, 255, 255))
    y = y0 + int(18 * s)
    for (text, font, color), h in zip(lines, heights):
        draw.text((x0 + int(30 * s), y), text, font=font, fill=color)
        y += h
    # elapsed time, top right
    clock = f"t = {max(t_rel, 0.0):5.1f} s"
    if args.speed_label:
        clock += f"   {args.speed_label}"
    f = _font(int(30 * s), True)
    w = draw.textlength(clock, font=f)
    cx1, cy0 = base.size[0] - int(48 * s), int(40 * s)
    draw.rounded_rectangle([cx1 - w - int(36 * s), cy0, cx1, cy0 + int(56 * s)], radius=int(12 * s), fill=(14, 18, 24, 160))
    draw.text((cx1 - w - int(18 * s), cy0 + int(10 * s)), clock, font=f, fill=(255, 255, 255, 255))
    # verdict once the program has ended
    if args.verdict and t_prog_end_rel is not None and t_rel >= t_prog_end_rel + 0.3:
        f = _font(int(32 * s), True)
        text = args.verdict
        w = draw.textlength(text, font=f)
        vx1, vy1 = base.size[0] - int(48 * s), base.size[1] - int(48 * s)
        draw.rounded_rectangle([vx1 - w - int(48 * s), vy1 - int(68 * s), vx1, vy1], radius=int(14 * s), fill=(22, 140, 72, 225))
        draw.text((vx1 - w - int(24 * s), vy1 - int(56 * s)), text, font=f, fill=(255, 255, 255, 255))
    return Image.alpha_composite(base, layer).convert("RGB")


_W: dict = {}


def _worker_init(args):
    import mujoco

    model = build_model(args.lab, args.width, args.height)
    t0, t1, t_end = window(args)
    jt, jq, jn, bodies = load_capture(Path(args.motion), t0, t1)
    _W["args"] = args
    _W["rep"] = Replay(model, jt, jq, jn, bodies)
    _W["ren"] = mujoco.Renderer(model, args.height, args.width)
    if args.pip:
        _W["pip"] = mujoco.Renderer(model, args.height // 3, args.width // 3) if args.width // 3 <= args.width else None
    _W["win"] = (t0, t1, t_end)


def window(args):
    if args.trace:
        t0, t1, t_end = trial_window(Path(args.trace), args.pre, args.post)
    else:
        t0, t1, t_end = args.t0, args.t1, None
    if args.t0 is not None:
        t0 = args.t0
    if args.t1 is not None:
        t1 = args.t1
    return t0, t1, t_end


def _scene_opts(ren):
    import mujoco

    scn = ren.scene
    scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1
    scn.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 1
    scn.flags[mujoco.mjtRndFlag.mjRND_HAZE] = 1
    scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0


def render_frame(i):
    from PIL import Image

    args, rep, ren = _W["args"], _W["rep"], _W["ren"]
    t0, t1, t_end = _W["win"]
    t = t0 + i / args.fps
    rep.set(t)
    duration = t1 - t0
    opt = None
    ren.update_scene(rep.d, camera=_camera(HERO[args.lab], t - t0, duration, args.drift), scene_option=opt)
    _scene_opts(ren)
    img = Image.fromarray(ren.render())
    if args.pip and _W.get("pip") is not None:
        pr = _W["pip"]
        preset = dict(CLOSE[args.lab])
        follow = preset.pop("follow", None)
        if follow and follow in rep.bodies:
            x, y = rep.bodies[follow][2][0][:2]  # where the body stood at the start of the window
            preset["lookat"] = (x, y, preset["lookat"][2])
        pr.update_scene(rep.d, camera=_camera(preset, 0, 0, 0))
        _scene_opts(pr)
        small = Image.fromarray(pr.render())
        from PIL import ImageDraw

        pad = int(6 * args.height / 1080)
        frame = Image.new("RGB", (small.size[0] + 2 * pad, small.size[1] + 2 * pad), (240, 242, 245))
        frame.paste(small, (pad, pad))
        img.paste(frame, (args.width - frame.size[0] - int(48 * args.height / 1080), int(120 * args.height / 1080)))
    img = overlay(img, args, t - (t0 + args.pre if args.trace else t0), None if t_end is None else t_end - (t0 + args.pre), args.height)
    out = Path(args.out) / "frames" / f"{i:06d}.jpg"
    img.save(out, quality=94, subsampling=0)
    return i


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lab", choices=sorted(LABS), required=True)
    ap.add_argument("--motion", required=True)
    ap.add_argument("--trace", help="the trial's trace.jsonl: its first and last ts set the window")
    ap.add_argument("--t0", type=float)
    ap.add_argument("--t1", type=float)
    ap.add_argument("--pre", type=float, default=1.0, help="seconds before the program's first call")
    ap.add_argument("--post", type=float, default=5.0, help="seconds after its last call (the judge's dwell)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="replay")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--drift", type=float, default=14.0, help="camera azimuth sweep, degrees either side")
    ap.add_argument("--pip", action="store_true", help="close-up picture-in-picture, top right")
    ap.add_argument("--title", default="")
    ap.add_argument("--program", default="")
    ap.add_argument("--score", default="")
    ap.add_argument("--verdict", default="", help="badge shown once the program has ended, e.g. 'Judge: PASS'")
    ap.add_argument("--speed-label", default="")
    ap.add_argument("--stills", default="", help="comma list of seconds into the window: render only these")
    ap.add_argument("--frames-only", action="store_true")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    args = ap.parse_args()

    t0, t1, t_end = window(args)
    (Path(args.out) / "frames").mkdir(parents=True, exist_ok=True)
    n = int((t1 - t0) * args.fps)
    if args.stills:
        idx = [int(float(s) * args.fps) for s in args.stills.split(",")]
    else:
        idx = list(range(n))
    print(f"window {t0:.2f}..{t1:.2f} ({t1 - t0:.1f} s), program end at {None if t_end is None else round(t_end - t0, 1)} s;"
          f" {len(idx)} frames on {args.workers} workers", flush=True)
    with Pool(args.workers, initializer=_worker_init, initargs=(args,)) as pool:
        for k, _ in enumerate(pool.imap_unordered(render_frame, idx, chunksize=4)):
            if k % 150 == 0:
                print(f"  {k}/{len(idx)}", flush=True)
    if args.stills or args.frames_only:
        return 0
    mp4 = Path(args.out) / f"{args.name}.mp4"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", str(args.fps), "-i",
                    str(Path(args.out) / "frames" / "%06d.jpg"), "-c:v", "libx264", "-preset", "slow", "-crf", "18",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)], check=True)
    print(mp4, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
