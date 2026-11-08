import math


def run(robot, task):
    """Tower in the order the task asks for, then the stick. The task text names the cubes bottom to top ("stack the
    yellow cube first, then the red cube on top, then the blue cube on top, then stand the red stick in the top
    cube's hole"); this parses the colours in order of appearance. Each level reuses stack_tower_v6's pick-and-place:
    finger pads aimed 7.8 cm ahead of the mount, absolute re-aims with a 0.7 gain against gravity sag, sag fed
    forward into the set-down, release-drag compensation along the jaw axis, a 35 percent jaw opening before the
    retreat, travel at 0.25 m with fallbacks, re-clearing the held object, verifying each level and rebuilding one
    that moved. The stick is grasped at its mid-height standing on the bench, carried over the top cube and lowered
    until its bottom is 2 cm inside the hole, then released; the cube's floor plate catches it.
    Expected failure: the stick needs 1 cm placement precision (hole 3 cm, stick 2 cm); a tilted top cube."""
    down_roll = math.pi
    pad = 0.078
    hover = 0.10
    travel_z = 0.25
    gain = 0.7
    drag = (0.009, -0.003)
    colours = ("yellow", "red", "blue")

    # --- what to build, from the task text ---
    words = [w.strip(".,;:'\"").lower() for w in task.text.split()]
    cubes, peg = [], None
    for i, w in enumerate(words):
        if w in colours and i + 1 < len(words):
            if words[i + 1] == "cube" and "cube_" + w not in cubes:
                cubes.append("cube_" + w)
            elif words[i + 1] in ("stick", "peg"):
                peg = "peg_" + w
    if len(cubes) < 2:
        cubes = ["cube_yellow", "cube_red", "cube_blue"]
    robot.log("order: %s then %s" % (" -> ".join(c.replace("cube_", "") for c in cubes), peg or "no stick"))

    def square_yaw(yaw):
        quarter = math.pi / 2
        return yaw - quarter * round(yaw / quarter)

    def find(name):
        for attempt in range(3):
            try:
                return robot.scene.objects([name])[0]
            except Exception as error:
                robot.log("%s not in the map yet (%s), retrying" % (name, error))
        return robot.scene.objects([name])[0]

    def dwell(reads=20):
        for _ in range(reads):
            robot.arm.pose()

    def reach(x, y, z, yaw, label, tries=3, xy_tol=0.004, z_tol=0.004, offset=(0.0, 0.0, 0.0)):
        cx, cy, cz = x + offset[0], y + offset[1], z + offset[2]
        robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
        p = robot.arm.pose().position
        for attempt in range(tries + 1):
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= xy_tol and abs(ez) <= z_tol:
                break
            if attempt == tries:
                robot.log("%s: still (%.1f, %.1f, %.1f) cm off after %d corrections" % (label, ex * 100, ey * 100, ez * 100, tries))
                break
            cx, cy, cz = cx + gain * ex, cy + gain * ey, cz + gain * ez
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
            p = robot.arm.pose().position
        return p, (cx - x, cy - y, cz - z)

    def travel_to(x, y, yaw):
        for z in (travel_z, travel_z - 0.03, travel_z - 0.06):
            try:
                robot.arm.move_pose([x, y, z], [down_roll, 0.0, yaw])
                return z
            except Exception as error:
                robot.log("travel to (%.2f, %.2f) at %.2f m failed (%s), lower" % (x, y, z, str(error)[:60]))
        raise RuntimeError("no reachable travel height over (%.2f, %.2f)" % (x, y))

    def go_high():
        p = robot.arm.pose()
        if p.position[2] < travel_z - 0.01:
            try:
                robot.arm.move_linear(dz=travel_z - p.position[2])
            except Exception as error:
                robot.log("straight rise failed (%s), planning it" % str(error)[:60])
                travel_to(p.position[0], p.position[1], 0.0)

    def seated(name, base_name):
        mover, base = find(name), find(base_name)
        off = math.hypot(mover.position[0] - base.position[0], mover.position[1] - base.position[1])
        dz = mover.position[2] - (base.position[2] + base.size[2] / 2 + mover.size[2] / 2)
        return off <= 0.015 and abs(dz) <= 0.008, off, dz

    def pick(name, grasp_depth=0.0):
        obj = find(name)
        ox, oy, oz = obj.position
        yaw = 0.0 if name.startswith("peg") else square_yaw(obj.yaw)
        robot.scene.clear(obj.id, margin=0.06)
        grasp_z = oz + pad - grasp_depth
        robot.gripper.open()
        go_high()
        travel_to(ox, oy, yaw)
        robot.arm.move_linear(dz=(grasp_z + hover) - robot.arm.pose().position[2])
        p, sag = reach(ox, oy, grasp_z + hover, yaw, "hover")
        at_close, sag = reach(ox, oy, grasp_z, yaw, "grasp", offset=sag)
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=hover)
        held = find(name)
        if held.position[2] - oz < 0.05:
            robot.log("lift of %s failed, retrying 5 mm deeper" % name)
            robot.gripper.open()
            dwell(10)
            at_close, sag = reach(ox, oy, grasp_z - 0.005, yaw, "regrasp", offset=sag)
            robot.gripper.close()
            dwell()
            robot.arm.move_linear(dz=hover + 0.005)
            held = find(name)
        in_hand = (ox - at_close[0], oy - at_close[1])
        robot.scene.clear(held.id, margin=0.08)
        robot.log("lifted %s %.1f cm; in-hand offset (%.1f, %.1f) cm" % (name, (held.position[2] - oz) * 100, in_hand[0] * 100, in_hand[1] * 100))
        return obj, yaw, in_hand

    def set_down(name, obj, yaw, in_hand, base_name, bottom_above_seat):
        # lower the held object over the base until its bottom sits bottom_above_seat above the base's top, release
        base = find(base_name)
        bx, by, bz = base.position
        robot.scene.clear(base.id, margin=0.06)
        dxw = drag[0] * math.cos(yaw) - drag[1] * math.sin(yaw)
        dyw = drag[0] * math.sin(yaw) + drag[1] * math.cos(yaw)
        tx, ty = bx - in_hand[0] - dxw, by - in_hand[1] - dyw
        base_top = bz + base.size[2] / 2
        tool_z = base_top + bottom_above_seat + obj.size[2] / 2 + pad  # object centre is a half-height above its bottom
        go_high()
        travel_to(tx, ty, yaw)
        robot.arm.move_linear(dz=(tool_z + 0.05) - robot.arm.pose().position[2])
        p, sag = reach(tx, ty, tool_z + 0.05, yaw, "over %s" % base_name)
        reach(tx, ty, tool_z, yaw, "set-down height", offset=sag)
        closed, opened = robot.gripper.range()
        robot.gripper.set(closed + 0.35 * (opened - closed))
        dwell()
        robot.arm.move_linear(dz=0.04)
        robot.gripper.open()
        dwell(10)
        robot.arm.move_linear(dz=0.04)

    def stack_cube(name, base_name):
        for attempt in range(2):
            obj, yaw, in_hand = pick(name)
            set_down(name, obj, yaw, in_hand, base_name, 0.006)
            ok, off, dz = seated(name, base_name)
            robot.log("placed %s %.1f cm off %s, %.1f cm from seat height" % (name, off * 100, base_name, dz * 100))
            if ok:
                return True
            robot.log("%s is not seated on %s, placing it again" % (name, base_name))
        return False

    def stand_stick(peg_name, top_name):
        obj, yaw, in_hand = pick(peg_name)
        # bottom 2 cm inside the hole: the cube's floor plate is at its centre, 2.5 cm below the top
        set_down(peg_name, obj, yaw, in_hand, top_name, -0.020)
        peg, top = find(peg_name), find(top_name)
        off = math.hypot(peg.position[0] - top.position[0], peg.position[1] - top.position[1])
        robot.log("stick %.1f cm off the top cube's axis, bottom %.1f cm above the bench" % (off * 100, (peg.position[2] - obj.size[2] / 2) * 100))

    for round_number in range(3):
        rebuilt = False
        for lower, upper in zip(cubes, cubes[1:]):
            ok, off, dz = seated(upper, lower)
            if ok:
                continue
            stack_cube(upper, lower)
            rebuilt = True
        if not rebuilt:
            break
    if peg:
        stand_stick(peg, cubes[-1])
    go_high()
