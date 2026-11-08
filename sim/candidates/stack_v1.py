import math


def run(robot, task):
    """Pick the red cube with a top-down grasp at its centre height, carry it 10 cm above the blue cube, lower it
    until it rests on the blue cube, release and retreat. Expected failure: the fingers close on the hole's rim and
    the cube tilts, or the red cube lands off-centre and slides off."""
    red = robot.scene.objects(["cube_red"])[0]
    blue = robot.scene.objects(["cube_blue"])[0]
    rx, ry, rz = red.position
    bx, by, bz = blue.position
    size = red.size[2]
    robot.scene.clear(red.id, margin=0.05)
    robot.scene.clear(blue.id, margin=0.05)
    down = [math.pi, 0.0, 0.0]

    robot.gripper.open()
    robot.arm.move_pose([rx, ry, rz + 0.12], down)
    robot.arm.move_linear(dz=-0.12)  # fingertips at the cube's centre height
    robot.gripper.close()
    robot.arm.move_linear(dz=0.15)
    robot.log("lifted the red cube")

    robot.arm.move_pose([bx, by, bz + size + 0.10], down)
    robot.arm.move_linear(dz=-0.09)  # red cube's bottom 1 cm above the blue cube's top
    robot.arm.move_linear(dz=-0.012)
    robot.gripper.open()
    robot.arm.move_linear(dz=0.10)
    robot.log("released on the blue cube")
