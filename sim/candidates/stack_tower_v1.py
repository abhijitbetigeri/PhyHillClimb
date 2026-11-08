import math


def run(robot, task):
    """Tower v1: the stack_v7 pick-and-place applied twice, red on blue then yellow on red. Every target is reached
    by move_pose plus re-aims with a 0.7 gain on the measured error (the arm sags 2-4 cm under gravity and a
    Cartesian move cannot cancel that), heights aim the finger pads (7.8 cm ahead of gripper_base), the jaw gets a
    dwell after each command, the carry is planned at the grasp yaw, and the release aims the tool at the base minus
    the in-hand offset measured when the jaw closed. The second level reads the red cube's resting place after the
    first and seats the yellow cube on it. Expected failure: the higher release (tool ~20 cm up) sags more and the
    corrections may need a third try; a corner grasp near 45 deg yaw."""
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

    def reach(x, y, z, yaw, label, tries=2, xy_tol=0.004, z_tol=0.006):
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
            cx, cy, cz = cx + gain * ex, cy + gain * ey, cz + ez
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
            p = robot.arm.pose().position
        return p

    def stack(mover_name, base_name):
        mover = find(mover_name)
        mx, my, mz = mover.position
        size = mover.size[2]
        yaw = square_yaw(mover.yaw)
        robot.log("%s at (%.3f, %.3f, %.3f) yaw %.1f deg -> grasp yaw %.1f deg" % (mover_name, mx, my, mz, math.degrees(mover.yaw), math.degrees(yaw)))
        robot.scene.clear(mover.id, margin=0.06)

        grasp_z = mz + pad - 0.005
        robot.gripper.open()
        reach(mx, my, grasp_z + hover, yaw, "hover")
        at_close = reach(mx, my, grasp_z, yaw, "grasp")
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=hover)
        held = find(mover_name)
        if held.position[2] - mz < 0.05:
            robot.log("lift failed, retrying")
            robot.gripper.open()
            dwell(10)
            at_close = reach(mx, my, grasp_z - 0.005, yaw, "regrasp")
            robot.gripper.close()
            dwell()
            robot.arm.move_linear(dz=hover + 0.005)
            held = find(mover_name)
        in_hand = (mx - at_close[0], my - at_close[1])
        robot.log("lifted %s %.1f cm; in-hand offset (%.1f, %.1f) cm" % (mover_name, (held.position[2] - mz) * 100, in_hand[0] * 100, in_hand[1] * 100))

        base = find(base_name)
        bx, by, bz = base.position
        robot.scene.clear(base.id, margin=0.06)
        tx, ty = bx - in_hand[0], by - in_hand[1]
        seat_tool_z = bz + base.size[2] / 2 + size / 2 + pad  # tool height when the mover rests on the base
        reach(tx, ty, seat_tool_z + 0.06, yaw, "over %s" % base_name)
        robot.arm.move_linear(dz=-0.05)
        reach(tx, ty, seat_tool_z + 0.01, yaw, "release height", z_tol=0.005)
        robot.gripper.open()
        dwell()
        robot.arm.move_linear(dz=0.08)
        placed = find(mover_name)
        off = math.hypot(placed.position[0] - bx, placed.position[1] - by)
        robot.log("placed %s %.1f cm off %s, centre %.1f cm above the bench" % (mover_name, off * 100, base_name, placed.position[2] * 100))
        return placed

    stack("cube_red", "cube_blue")
    stack("cube_yellow", "cube_red")
