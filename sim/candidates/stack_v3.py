import math


def run(robot, task):
    """v3: align the jaws with the cube's faces. Keyframes of v1 and v2 show the fingers closing on the red cube's
    corners whenever its yaw differs from the gripper's, so it twists and slips out on the lift. The scene now reports
    each object's yaw; reduce it to the nearest quarter turn (a cube is symmetric) and grasp with that yaw. Release
    1 cm above the seat. Expected failure: a yaw near 45 degrees is ambiguous and may still land on a corner."""
    down_roll = math.pi

    def square_yaw(yaw):
        # the cube's face direction nearest to the gripper's zero yaw, in (-45, 45] degrees
        quarter = math.pi / 2
        return yaw - quarter * round(yaw / quarter)

    red = robot.scene.objects(["cube_red"])[0]
    rx, ry, rz = red.position
    size = red.size[2]
    yaw = square_yaw(red.yaw)
    robot.log("red yaw %.1f deg -> grasp yaw %.1f deg" % (math.degrees(red.yaw), math.degrees(yaw)))
    robot.scene.clear(red.id, margin=0.05)

    robot.gripper.open()
    robot.arm.move_pose([rx, ry, rz + 0.12], [down_roll, 0.0, yaw])
    robot.arm.move_linear(dz=-0.12)
    robot.gripper.close()
    robot.arm.move_linear(dz=0.15)
    held = robot.scene.objects(["cube_red"])[0]
    if held.position[2] - rz < 0.08:
        robot.log("lift failed, retrying with a slightly lower grasp")
        robot.gripper.open()
        robot.arm.move_linear(dz=-0.15)
        robot.arm.move_linear(dz=-0.005)
        robot.gripper.close()
        robot.arm.move_linear(dz=0.155)
        held = robot.scene.objects(["cube_red"])[0]
    robot.log("lifted %.1f cm" % ((held.position[2] - rz) * 100))

    blue = robot.scene.objects(["cube_blue"])[0]
    bx, by, bz = blue.position
    robot.scene.clear(blue.id, margin=0.05)
    tool = robot.arm.pose()
    dx, dy = tool.position[0] - held.position[0], tool.position[1] - held.position[1]
    seat = bz + size
    robot.arm.move_pose([bx + dx, by + dy, seat + 0.10], [down_roll, 0.0, square_yaw(blue.yaw)])
    robot.arm.move_linear(dz=-0.09)
    robot.gripper.open()
    robot.arm.move_linear(dz=0.12)
    placed = robot.scene.objects(["cube_red"])[0]
    robot.log("placed %.1f cm off the blue axis" % (math.hypot(placed.position[0] - bx, placed.position[1] - by) * 100))
