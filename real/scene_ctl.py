"""Talk to the running stack's measured scene from a shell, between trials.

    python -m real.scene_ctl describe                 # believed positions, what the jaw holds, last grasp events
    python -m real.scene_ctl reload                   # re-read the YAML (after editing it); clears the grasp belief
    python -m real.scene_ctl reload real/scene_cable.yaml   # switch to another scene file
    python -m real.scene_ctl set cube_red 0.25 0.10   # one object standing on the bench at (x, y), no file edit
    python -m real.scene_ctl pose                     # the arm's tool (gripper_base) pose and joints, read-only
    python -m real.scene_ctl jog 1 5 --go             # MOVES: joint1 by +5 degrees at speed 0.05 (planned)
    python -m real.scene_ctl home --go                # MOVES: planned move to the home preset at speed 0.05
    python -m real.scene_ctl rest --go                # MOVES: planned move to all joints 0 (the folded rest pose)

Run it with the dimos venv's python from any directory; it attaches over the bus like `dimos shell`.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from dimos.porcelain.dimos import Dimos


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("describe")
    sub.add_parser("reload").add_argument("path", nargs="?")
    sub.add_parser("pose")
    j = sub.add_parser("jog")
    j.add_argument("joint", type=int, help="1..6")
    j.add_argument("degrees", type=float)
    for p in (j, sub.add_parser("home"), sub.add_parser("rest")):
        p.add_argument("--speed", type=float, default=0.05)
        p.add_argument("--go", action="store_true", help="required: this moves the arm")
    s = sub.add_parser("set")
    s.add_argument("name")
    s.add_argument("x", type=float)
    s.add_argument("y", type=float)
    s.add_argument("z", type=float, nargs="?")
    args = parser.parse_args()
    app = Dimos.connect()
    try:
        if args.cmd in ("jog", "home", "rest"):
            from dimos.manipulation.sdk import Arm

            if not args.go:
                raise SystemExit("this moves the arm: hand on the e-stop, then add --go")
            arm = Arm.from_app(app)
            if args.cmd == "home":
                arm.home(speed_scale=args.speed)
            elif args.cmd == "rest":
                arm.move_joints([0.0] * 6, speed_scale=args.speed)
            else:
                if not 1 <= args.joint <= 6 or abs(args.degrees) > 20:
                    raise SystemExit("joint 1..6 and at most 20 degrees per jog")
                target = list(arm.joints())
                target[args.joint - 1] += math.radians(args.degrees)
                arm.move_joints(target, speed_scale=args.speed)
            print("joints now (deg):", ", ".join(f"{math.degrees(v):.1f}" for v in arm.joints()))
            return
        if args.cmd == "pose":
            from dimos.manipulation.sdk import Arm

            arm = Arm.from_app(app)
            pose = arm.pose()
            p, q = pose.position, pose.orientation
            rpy = q.to_euler()
            print(f"gripper_base xyz = ({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m")
            print(f"  pads (7.8 cm along the approach) ~ z {p.z - 0.078:.4f} m when pointing down")
            print(f"  rpy = ({rpy.x:.3f}, {rpy.y:.3f}, {rpy.z:.3f}) rad")
            print("joints (rad):", ", ".join(f"{v:.4f}" for v in arm.joints()))
            print("joints (deg):", ", ".join(f"{math.degrees(v):.1f}" for v in arm.joints()))
            return
        scene = app.get_module("MeasuredSceneModule")
        if args.cmd == "describe":
            print(scene.describe())
        elif args.cmd == "reload":
            print(scene.reload(str(Path(args.path).resolve())) if args.path else scene.reload())
        elif args.cmd == "set":
            print(scene.set_object(args.name, args.x, args.y, args.z))
    finally:
        app.stop()


if __name__ == "__main__":
    main()
