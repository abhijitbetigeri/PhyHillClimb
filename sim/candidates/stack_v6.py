import math


def run(robot, task):
    """v6: absolute pose corrections and a planned carry at the grasp yaw. v5 showed the arm sags 2-4 cm below any
    commanded height (the sim reproduces the real arm's gravity sag: no gravity compensation, torque over kp), and
    that move_linear cannot cancel it: a Cartesian move plans from the measured pose, so "move up by the error"
    re-commands the same height. Corrections now re-issue move_pose with the error added to the target (integral
    action on the command), which converges because the goal is absolute. v5's straight 18 cm carry also hit a
    wrist singularity in two trials and dropped the cube in one; the carry is a planned move_pose kept at the grasp
    yaw, followed by a correction. Lifts are shorter (10 cm) to stay clear of the singular region above the bench.
    Expected failure: a yaw near 45 degrees still gives a corner grasp; the planned carry may still swing hard."""
    down_roll = math.pi
    pad = 0.078  # finger-pad centre ahead of gripper_base, the frame pose()/move_pose address
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
        for _ in range(reads):  # ~30 ms per read: lets the jaw finish moving (gripper calls do not wait)
            robot.arm.pose()

    def reach(x, y, z, yaw, label, tries=2, xy_tol=0.005, z_tol=0.008):
        # planned move, then re-plan to target + accumulated error until the measured pose is within tolerance
        cx, cy, cz = x, y, z
        robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
        for attempt in range(tries + 1):
            p = robot.arm.pose().position
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= xy_tol and abs(ez) <= z_tol:
                robot.log("%s: within tolerance after %d corrections" % (label, attempt))
                return
            if attempt == tries:
                robot.log("%s: still (%.1f, %.1f, %.1f) cm off after %d corrections" % (label, ex * 100, ey * 100, ez * 100, tries))
                return
            cx, cy, cz = cx + ex, cy + ey, cz + ez
            robot.log("%s: off by (%.1f, %.1f, %.1f) cm, re-aiming" % (label, ex * 100, ey * 100, ez * 100))
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])

    red = find("cube_red")
    rx, ry, rz = red.position
    size = red.size[2]
    yaw = square_yaw(red.yaw)
    robot.log("red at (%.3f, %.3f) yaw %.1f deg -> grasp yaw %.1f deg" % (rx, ry, math.degrees(red.yaw), math.degrees(yaw)))
    robot.scene.clear(red.id, margin=0.06)

    grasp_z = rz + pad - 0.005  # pads just below the cube's centre height; fingertips ~0.5 cm above the bench
    robot.gripper.open()
    reach(rx, ry, grasp_z + hover, yaw, "hover")
    reach(rx, ry, grasp_z, yaw, "grasp", tries=1)
    robot.gripper.close()
    dwell()
    robot.arm.move_linear(dz=hover)
    held = find("cube_red")
    if held.position[2] - rz < 0.05:
        robot.log("lift failed, retrying")
        robot.gripper.open()
        dwell(10)
        reach(rx, ry, grasp_z - 0.005, yaw, "regrasp", tries=1)
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=hover + 0.005)
        held = find("cube_red")
    robot.log("lifted %.1f cm" % ((held.position[2] - rz) * 100))

    blue = find("cube_blue")
    bx, by, bz = blue.position
    robot.scene.clear(blue.id, margin=0.06)
    seat_tool_z = bz + size + pad  # gripper_base height when the red cube rests on the blue one
    reach(bx, by, seat_tool_z + 0.06, yaw, "over blue")  # cube bottom ~6 cm above the blue top, same yaw as the grasp
    robot.arm.move_linear(dz=-0.05)
    reach(bx, by, seat_tool_z + 0.01, yaw, "release height", tries=1, z_tol=0.005)  # cube bottom ~1 cm above the top
    robot.gripper.open()
    dwell()
    robot.arm.move_linear(dz=0.08)
    placed = find("cube_red")
    robot.log("placed %.1f cm off the blue axis, %.1f cm above the bench" % (
        math.hypot(placed.position[0] - bx, placed.position[1] - by) * 100, placed.position[2] * 100))
