import math


def run(robot, task):
    """Tower v6: tower v5 plus a re-clear of the held cube. Held-out trial 0 of v5 died on the carry with "Start
    configuration is in collision": the cube was cleared from the obstacle map around its starting box (6 cm margin),
    and after a 10 cm lift it sits outside that zone, so the map treats the cube in the jaw as an obstacle touching
    the fingers. The cube is now cleared again wherever the scene reports it after the lift (clears add up).
    Tower v5: tower v4 with a reachable travel height. v4 seated level 1 in 4/4 completed trials (0.8-1.4 cm,
    the drag compensation works) and then every trial died on the way to the yellow cube: move_pose to (0.34, 0.14)
    at 0.30 m with the jaw pointing down has no IK solution on the Metal (that pose at 0.24 m was fine in tower v1),
    and one go_high move_linear hit a singularity near the top of the reach. Travel height is now 0.25 m, every
    travel-height move falls back 3 cm at a time when planning fails, and go_high falls back to a planned move.
    Tower v4: tower v3 plus release-drag compensation. Across every seated placement of v6/v7/tower v1-v3 the
    cube ended 0.6-1.0 cm from where the tool axis released it, always along the jaw's opening axis in the tool
    frame (+0.9 cm, -0.3 cm): one pad drags the cube as the jaw opens. The release now aims the tool at the base
    minus that drag (rotated by the grasp yaw) minus the in-hand offset, and the jaw opens only 35 percent before the
    retreat, then fully. The seated check matches the judge (1.5 cm, 0.8 cm) so a good level is not rebuilt.
    From tower v3: keep the arm away from the tower it has built. Tower v1/v2 keyframes show the red cube seated on the
    blue one, then knocked 11 cm away while the arm went for the yellow cube: scene.clear() leaves the red cube out
    of the obstacle map for the rest of the trial, so the planner is free to swing the jaw and forearm through the
    tower on the way to the next cube and on the carry back. Changes: every travel between a release and the next
    cube, and every carry, goes through a travel height (gripper_base at 0.30 m, fingertips 8 cm above a two-level
    tower); descents are straight move_linear moves before the re-aim; before each level the program checks the
    lower levels are still seated and rebuilds any that moved; a level is placed again if it ends more than 1.2 cm
    off its base. Tower v2 also showed the first set-down command landing 3 cm low (gravity sag) and pressing the
    stack, so each reach feeds the sag learned at the reach just above it forward; set-down 6 mm above the seat.
    Expected failure: the planned move at travel height can still dip an elbow into the tower; corner grasps."""
    down_roll = math.pi
    pad = 0.078  # finger-pad centre ahead of gripper_base, the frame pose()/move_pose address
    hover = 0.10
    travel_z = 0.25
    gain = 0.7
    drag = (0.009, -0.003)  # where the cube ends relative to the tool axis after the jaw opens, tool frame, metres

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
        for _ in range(reads):  # ~30 ms per read: lets the jaw finish moving (gripper calls do not wait)
            robot.arm.pose()

    def reach(x, y, z, yaw, label, tries=3, xy_tol=0.004, z_tol=0.004, offset=(0.0, 0.0, 0.0)):
        # planned move to the target plus a feed-forward offset (the sag learned at the previous reach nearby), then
        # re-aims with gain on the measured error; returns the measured pose and the offset that got there
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
        # a planned move at travel height; the Metal cannot point the jaw down everywhere at 0.25 m, so step down
        for z in (travel_z, travel_z - 0.03, travel_z - 0.06):
            try:
                robot.arm.move_pose([x, y, z], [down_roll, 0.0, yaw])
                return z
            except Exception as error:
                robot.log("travel to (%.2f, %.2f) at %.2f m failed (%s), lower" % (x, y, z, str(error)[:60]))
        raise RuntimeError("no reachable travel height over (%.2f, %.2f)" % (x, y))

    def go_high():
        p = robot.arm.pose()
        z = p.position[2]
        if z < travel_z - 0.01:
            try:
                robot.arm.move_linear(dz=travel_z - z)
            except Exception as error:
                robot.log("straight rise failed (%s), planning it" % str(error)[:60])
                travel_to(p.position[0], p.position[1], 0.0)

    def seated(name, base_name):
        mover, base = find(name), find(base_name)
        off = math.hypot(mover.position[0] - base.position[0], mover.position[1] - base.position[1])
        dz = mover.position[2] - (base.position[2] + base.size[2] / 2 + mover.size[2] / 2)
        return off <= 0.015 and abs(dz) <= 0.008, off, dz

    def pick(name):
        mover = find(name)
        mx, my, mz = mover.position
        yaw = square_yaw(mover.yaw)
        robot.scene.clear(mover.id, margin=0.06)
        grasp_z = mz + pad  # pads at the cube's centre height; fingertips ~1 cm above the bench
        robot.gripper.open()
        go_high()
        travel_to(mx, my, yaw)  # arrive from above
        robot.arm.move_linear(dz=(grasp_z + hover) - robot.arm.pose().position[2])
        p, sag = reach(mx, my, grasp_z + hover, yaw, "hover")
        at_close, sag = reach(mx, my, grasp_z, yaw, "grasp", offset=sag)
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=hover)
        held = find(name)
        if held.position[2] - mz < 0.05:
            robot.log("lift of %s failed, retrying 5 mm deeper" % name)
            robot.gripper.open()
            dwell(10)
            at_close, sag = reach(mx, my, grasp_z - 0.005, yaw, "regrasp", offset=sag)
            robot.gripper.close()
            dwell()
            robot.arm.move_linear(dz=hover + 0.005)
            held = find(name)
        in_hand = (mx - at_close[0], my - at_close[1])
        robot.scene.clear(held.id, margin=0.08)  # the lifted cube left its cleared zone: clear it where it is now
        robot.log("lifted %s %.1f cm; in-hand offset (%.1f, %.1f) cm" % (name, (held.position[2] - mz) * 100, in_hand[0] * 100, in_hand[1] * 100))
        return mover, yaw, in_hand

    def place(name, mover, yaw, in_hand, base_name):
        base = find(base_name)
        bx, by, bz = base.position
        robot.scene.clear(base.id, margin=0.06)
        dxw = drag[0] * math.cos(yaw) - drag[1] * math.sin(yaw)
        dyw = drag[0] * math.sin(yaw) + drag[1] * math.cos(yaw)
        tx, ty = bx - in_hand[0] - dxw, by - in_hand[1] - dyw
        seat_tool_z = bz + base.size[2] / 2 + mover.size[2] / 2 + pad  # tool height when the mover rests on the base
        go_high()
        travel_to(tx, ty, yaw)  # carry at travel height
        robot.arm.move_linear(dz=(seat_tool_z + 0.05) - robot.arm.pose().position[2])
        p, sag = reach(tx, ty, seat_tool_z + 0.05, yaw, "over %s" % base_name)
        reach(tx, ty, seat_tool_z + 0.006, yaw, "set-down height", offset=sag)  # cube bottom 6 mm above the seat; the
        # sag learned 5 cm higher is fed forward so the first command no longer lands 3 cm low and presses the stack
        closed, opened = robot.gripper.range()
        robot.gripper.set(closed + 0.35 * (opened - closed))  # free the cube with the least pad travel
        dwell()
        robot.arm.move_linear(dz=0.04)
        robot.gripper.open()
        dwell(10)
        robot.arm.move_linear(dz=0.04)
        ok, off, dz = seated(name, base_name)
        robot.log("placed %s %.1f cm off %s, %.1f cm from seat height" % (name, off * 100, base_name, dz * 100))
        return ok

    def stack(name, base_name):
        for attempt in range(2):
            mover, yaw, in_hand = pick(name)
            if place(name, mover, yaw, in_hand, base_name):
                return True
            robot.log("%s is not seated on %s, placing it again" % (name, base_name))
        return False

    levels = [("cube_red", "cube_blue"), ("cube_yellow", "cube_red")]
    for round_number in range(3):
        rebuilt = False
        for name, base_name in levels:
            ok, off, dz = seated(name, base_name)
            if ok:
                continue
            if off > 0.0 and round_number > 0:
                robot.log("%s is %.1f cm off %s, rebuilding that level" % (name, off * 100, base_name))
            stack(name, base_name)
            rebuilt = True
        if not rebuilt:
            break
    go_high()
