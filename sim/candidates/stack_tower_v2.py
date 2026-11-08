import math


def run(robot, task):
    """Tower v2: set the cube down instead of dropping it, and verify-then-retry each level. Tower v1 traces: the
    red level seated in 4/5, but every yellow release happened with the cube's bottom ~1.7 cm above the red cube
    (the z re-aim overshoots at that height, the arm ended 0.5-1 cm ABOVE the target) and the yellow tumbled off a
    red cube that itself sat 0.6 cm high (perched after its own drop). Changes: the re-aim gain of 0.7 now applies to
    z too, with three tries and 4 mm tolerances; the release height puts the cube's bottom 4 mm above the seat; after
    each release the program reads where the cube ended and, if it is more than 1.2 cm off its base or not at seat
    height, picks it up again from wherever it is and places it once more (one retry per level). Expected failure:
    a retry that knocks the lower level; corner grasps near 45 deg yaw."""
    down_roll = math.pi
    pad = 0.078  # finger-pad centre ahead of gripper_base, the frame pose()/move_pose address
    hover = 0.10
    gain = 0.7

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

    def reach(x, y, z, yaw, label, tries=3, xy_tol=0.004, z_tol=0.004):
        cx, cy, cz = x, y, z
        robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
        p = robot.arm.pose().position
        for attempt in range(tries + 1):
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= xy_tol and abs(ez) <= z_tol:
                return p
            if attempt == tries:
                robot.log("%s: still (%.1f, %.1f, %.1f) cm off after %d corrections" % (label, ex * 100, ey * 100, ez * 100, tries))
                return p
            cx, cy, cz = cx + gain * ex, cy + gain * ey, cz + gain * ez
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
            p = robot.arm.pose().position
        return p

    def pick(name):
        mover = find(name)
        mx, my, mz = mover.position
        yaw = square_yaw(mover.yaw)
        robot.scene.clear(mover.id, margin=0.06)
        grasp_z = mz + pad - 0.005
        robot.gripper.open()
        reach(mx, my, grasp_z + hover, yaw, "hover")
        at_close = reach(mx, my, grasp_z, yaw, "grasp")
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=hover)
        held = find(name)
        if held.position[2] - mz < 0.05:
            robot.log("lift of %s failed, retrying 5 mm deeper" % name)
            robot.gripper.open()
            dwell(10)
            at_close = reach(mx, my, grasp_z - 0.005, yaw, "regrasp")
            robot.gripper.close()
            dwell()
            robot.arm.move_linear(dz=hover + 0.005)
            held = find(name)
        in_hand = (mx - at_close[0], my - at_close[1])
        robot.log("lifted %s %.1f cm; in-hand offset (%.1f, %.1f) cm" % (name, (held.position[2] - mz) * 100, in_hand[0] * 100, in_hand[1] * 100))
        return mover, yaw, in_hand

    def place(name, mover, yaw, in_hand, base_name):
        base = find(base_name)
        bx, by, bz = base.position
        robot.scene.clear(base.id, margin=0.06)
        tx, ty = bx - in_hand[0], by - in_hand[1]
        seat_tool_z = bz + base.size[2] / 2 + mover.size[2] / 2 + pad  # tool height when the mover rests on the base
        reach(tx, ty, seat_tool_z + 0.06, yaw, "over %s" % base_name)
        robot.arm.move_linear(dz=-0.05)
        reach(tx, ty, seat_tool_z + 0.004, yaw, "set-down height")  # cube bottom 4 mm above the seat
        robot.gripper.open()
        dwell()
        robot.arm.move_linear(dz=0.08)
        placed = find(name)
        off = math.hypot(placed.position[0] - bx, placed.position[1] - by)
        dz = placed.position[2] - (bz + base.size[2] / 2 + mover.size[2] / 2)
        robot.log("placed %s %.1f cm off %s, %.1f cm from seat height" % (name, off * 100, base_name, dz * 100))
        return off <= 0.012 and abs(dz) <= 0.006

    def stack(name, base_name):
        for attempt in range(2):
            mover, yaw, in_hand = pick(name)
            if place(name, mover, yaw, in_hand, base_name):
                return True
            robot.log("%s is not seated on %s, placing it again" % (name, base_name))
        return False

    stack("cube_red", "cube_blue")
    stack("cube_yellow", "cube_red")
