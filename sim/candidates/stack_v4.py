import math


def run(robot, task):
    """v4: aim the finger pads, not the wrist mount. arm.pose()/move_pose() address gripper_base, the jaw's mount;
    the pads that close on an object sit about 7.8 cm further along the approach (dimos METAL_HEURISTIC_AIM). v1-v3
    drove the mount to the cube's centre height, so the pads were asked to go through the bench, the descent was cut
    short and the jaw closed on the cube's top edge (slips on the lift) and the release pressed the cube into the one
    below (slides off). v4 adds the pad offset everywhere, places without the in-hand offset correction of v3 (the
    held cube is half hidden by the jaw, its reported position drifted by 1 cm), and retries a missing detection.
    Expected failure: a constant x/y bias between the mount axis and the pads shows up as a repeatable offset."""
    down_roll = math.pi
    pad = 0.078  # pad centre ahead of the mount along the approach, metres
    hover = 0.10

    def square_yaw(yaw):
        quarter = math.pi / 2
        return yaw - quarter * round(yaw / quarter)

    def find(name):
        for attempt in range(3):
            try:
                return robot.scene.objects([name])[0]
            except Exception as error:  # the map can lag behind the camera; objects() itself waits 2 s per call
                robot.log("%s not in the map yet (%s), retrying" % (name, error))
        return robot.scene.objects([name])[0]

    red = find("cube_red")
    rx, ry, rz = red.position
    size = red.size[2]
    yaw = square_yaw(red.yaw)
    robot.log("red at (%.3f, %.3f) yaw %.1f deg -> grasp yaw %.1f deg" % (rx, ry, math.degrees(red.yaw), math.degrees(yaw)))
    robot.scene.clear(red.id, margin=0.05)

    grasp_z = rz + pad  # pads at the cube's centre height; fingertips stay ~1 cm above the bench
    robot.gripper.open()
    robot.arm.move_pose([rx, ry, grasp_z + hover], [down_roll, 0.0, yaw])
    robot.arm.move_linear(dz=-hover)
    robot.gripper.close()
    robot.arm.move_linear(dz=hover + 0.03)
    held = find("cube_red")
    if held.position[2] - rz < 0.06:
        robot.log("lift failed, retrying 1 cm deeper")
        robot.gripper.open()
        robot.arm.move_linear(dz=-(hover + 0.03) - 0.01)
        robot.gripper.close()
        robot.arm.move_linear(dz=hover + 0.04)
        held = find("cube_red")
    tool = robot.arm.pose()
    robot.log("lifted %.1f cm; in-hand offset reads (%.1f, %.1f) cm, not applied" % (
        (held.position[2] - rz) * 100,
        (held.position[0] - tool.position[0]) * 100, (held.position[1] - tool.position[1]) * 100))

    blue = find("cube_blue")
    bx, by, bz = blue.position
    robot.scene.clear(blue.id, margin=0.05)
    seat_tool_z = bz + size + pad  # mount height when the red cube rests on the blue one
    robot.arm.move_pose([bx, by, seat_tool_z + 0.08], [down_roll, 0.0, square_yaw(blue.yaw)])
    robot.arm.move_linear(dz=-0.07)  # red cube's bottom 1 cm above the blue cube's top: drop, never press
    robot.gripper.open()
    robot.arm.move_linear(dz=0.12)
    placed = find("cube_red")
    robot.log("placed %.1f cm off the blue axis, %.1f cm above the bench" % (
        math.hypot(placed.position[0] - bx, placed.position[1] - by) * 100, placed.position[2] * 100))
