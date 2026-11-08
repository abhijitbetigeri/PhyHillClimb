import math


def run(robot, task):
    """Top-down grasp at the cube's centre, then lift 12 cm. May fail if the fingers land on an edge."""
    cube = robot.scene.objects(["cube"])[0]
    x, y, z = cube.position
    robot.scene.clear(cube.id)
    robot.gripper.open()
    robot.arm.move_pose([x, y, z + 0.12], [math.pi, 0.0, 0.0])
    robot.arm.move_linear(dz=-0.09)
    robot.gripper.close()
    robot.arm.move_linear(dz=0.12)
    robot.log("lifted")
