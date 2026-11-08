import math


def run(robot, task):
    """v7: v6 (4/5) with the two residuals removed. Its traces show (a) the x/y correction overshoots: a +1.1 cm
    re-aim moved the tool +1.6 cm, leaving -0.5 cm, so corrections now use a 0.7 gain and one more try; (b) the
    cube sits 0.6-0.9 cm off the tool axis in the jaw, exactly the tool's residual error at the moment the jaw
    closed, so the release aims the tool at the blue cube minus that in-hand offset (measured before the grasp, from
    the unoccluded cube reading, not from the half-hidden held cube). Expected failure: corner grasps near 45 deg."""
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
        # planned move, then re-plan to the command plus gain * error until the measured pose is within tolerance
        cx, cy, cz = x, y, z
        robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
        p = robot.arm.pose().position
        for attempt in range(tries + 1):
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= xy_tol and abs(ez) <= z_tol:
                robot.log("%s: within tolerance after %d corrections" % (label, attempt))
                return p
            if attempt == tries:
                robot.log("%s: still (%.1f, %.1f, %.1f) cm off after %d corrections" % (label, ex * 100, ey * 100, ez * 100, tries))
                return p
            cx, cy, cz = cx + gain * ex, cy + gain * ey, cz + ez
            robot.log("%s: off by (%.1f, %.1f, %.1f) cm, re-aiming" % (label, ex * 100, ey * 100, ez * 100))
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
            p = robot.arm.pose().position
        return p

    red = find("cube_red")
    rx, ry, rz = red.position
    size = red.size[2]
    yaw = square_yaw(red.yaw)
    robot.log("red at (%.3f, %.3f) yaw %.1f deg -> grasp yaw %.1f deg" % (rx, ry, math.degrees(red.yaw), math.degrees(yaw)))
    robot.scene.clear(red.id, margin=0.06)

    grasp_z = rz + pad - 0.005  # pads just below the cube's centre height; fingertips ~0.5 cm above the bench
    robot.gripper.open()
    reach(rx, ry, grasp_z + hover, yaw, "hover")
    at_close = reach(rx, ry, grasp_z, yaw, "grasp")
    robot.gripper.close()
    dwell()
    robot.arm.move_linear(dz=hover)
    held = find("cube_red")
    if held.position[2] - rz < 0.05:
        robot.log("lift failed, retrying")
        robot.gripper.open()
        dwell(10)
        at_close = reach(rx, ry, grasp_z - 0.005, yaw, "regrasp")
        robot.gripper.close()
        dwell()
        robot.arm.move_linear(dz=hover + 0.005)
        held = find("cube_red")
    in_hand = (rx - at_close[0], ry - at_close[1])  # where the cube sits relative to the tool axis
    robot.log("lifted %.1f cm; cube is (%.1f, %.1f) cm off the tool axis" % ((held.position[2] - rz) * 100, in_hand[0] * 100, in_hand[1] * 100))

    blue = find("cube_blue")
    bx, by, bz = blue.position
    robot.scene.clear(blue.id, margin=0.06)
    tx, ty = bx - in_hand[0], by - in_hand[1]  # tool target that centres the held cube on the blue one
    seat_tool_z = bz + size + pad  # gripper_base height when the red cube rests on the blue one
    reach(tx, ty, seat_tool_z + 0.06, yaw, "over blue")
    robot.arm.move_linear(dz=-0.05)
    reach(tx, ty, seat_tool_z + 0.01, yaw, "release height", z_tol=0.005)  # cube bottom ~1 cm above the top
    robot.gripper.open()
    dwell()
    robot.arm.move_linear(dz=0.08)
    placed = find("cube_red")
    robot.log("placed %.1f cm off the blue axis, %.1f cm above the bench" % (
        math.hypot(placed.position[0] - bx, placed.position[1] - by) * 100, placed.position[2] * 100))
