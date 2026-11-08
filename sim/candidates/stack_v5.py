import math


def run(robot, task):
    """v5: close the loop on the arm's own pose and carry in a straight line. v4's traces show (a) the arm settles
    1.5-3 cm in x and up to 4 cm in z away from a move_pose target (gravity sag of the position-controlled joints), so
    the jaw landed on the cube's edge in dev-01; (b) gripper.close returns in 13 ms without waiting for the jaw, and
    (c) the planned carry (move_pose, a joint-space swing with a yaw change) threw the held cube off in dev-00.
    Every target is now reached by move_pose followed by read-pose-and-correct with move_linear, the jaw gets a
    dwell of pose reads after closing, and the carry is a straight horizontal move_linear at the grasp orientation.
    Expected failure: a cube whose yaw is near 45 degrees still gets a corner grasp."""
    down_roll = math.pi
    pad = 0.078  # finger-pad centre ahead of gripper_base (the frame pose()/move_pose address) along the approach
    hover = 0.10

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
        # no time module: ~30 ms per pose read gives the jaw ~0.6 s to finish moving
        for _ in range(reads):
            robot.arm.pose()

    def settle(x, y, z, label, tries=2, xy_tol=0.004, z_tol=0.006):
        # move_linear from the measured pose until gripper_base is within tolerance of the target
        for attempt in range(tries + 1):
            p = robot.arm.pose().position
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= xy_tol and abs(ez) <= z_tol:
                return
            if attempt == tries:
                robot.log("%s: still (%.1f, %.1f, %.1f) cm off after %d corrections" % (label, ex * 100, ey * 100, ez * 100, tries))
                return
            robot.log("%s: correcting (%.1f, %.1f, %.1f) cm" % (label, ex * 100, ey * 100, ez * 100))
            robot.arm.move_linear(dx=ex, dy=ey, dz=ez)

    red = find("cube_red")
    rx, ry, rz = red.position
    size = red.size[2]
    yaw = square_yaw(red.yaw)
    robot.log("red at (%.3f, %.3f) yaw %.1f deg -> grasp yaw %.1f deg" % (rx, ry, math.degrees(red.yaw), math.degrees(yaw)))
    robot.scene.clear(red.id, margin=0.05)

    grasp_z = rz + pad  # pads at the cube's centre height, fingertips ~1 cm above the bench
    robot.gripper.open()
    robot.arm.move_pose([rx, ry, grasp_z + hover], [down_roll, 0.0, yaw])
    settle(rx, ry, grasp_z + hover, "hover")
    robot.arm.move_linear(dz=-hover)
    settle(rx, ry, grasp_z, "grasp")
    robot.gripper.close()
    dwell()
    carry_z = grasp_z + hover + 0.05
    robot.arm.move_linear(dz=carry_z - grasp_z)
    held = find("cube_red")
    if held.position[2] - rz < 0.06:
        robot.log("lift failed, retrying")
        robot.gripper.open()
        dwell(10)
        robot.arm.move_linear(dz=grasp_z - 0.01 - carry_z)
        settle(rx, ry, grasp_z - 0.01, "regrasp")
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=carry_z - (grasp_z - 0.01))
        held = find("cube_red")
    robot.log("lifted %.1f cm" % ((held.position[2] - rz) * 100))

    blue = find("cube_blue")
    bx, by, bz = blue.position
    robot.scene.clear(blue.id, margin=0.05)
    seat_tool_z = bz + size + pad  # gripper_base height when the red cube rests on the blue one
    p = robot.arm.pose().position
    robot.arm.move_linear(dx=bx - p[0], dy=by - p[1], dz=0.0)  # straight carry, orientation unchanged
    settle(bx, by, p[2], "over blue")
    robot.arm.move_linear(dz=(seat_tool_z + 0.012) - robot.arm.pose().position[2])  # cube bottom ~1 cm above the blue top
    settle(bx, by, seat_tool_z + 0.012, "release height", z_tol=0.004)
    robot.gripper.open()
    dwell()
    robot.arm.move_linear(dz=0.12)
    placed = find("cube_red")
    robot.log("placed %.1f cm off the blue axis, %.1f cm above the bench" % (
        math.hypot(placed.position[0] - bx, placed.position[1] - by) * 100, placed.position[2] * 100))
