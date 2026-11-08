import math


def run(robot, task):
    """v2 of the cube stack. Trace of v1: the grasp and lift worked (wrist keyframes show the red cube held),
    but the cube ended on the bench 2.7 cm off the blue cube's axis, so it slid off after release. Changes:
    re-read the blue cube's pose right before the carry, lower in two steps and release 0.5 cm above the seat
    instead of pressing down 1.2 cm, then lift straight up before any sideways motion. Expected failure: a cube
    held off-centre in the fingers still lands off-centre; if so, v3 should centre the grasp from the wrist view."""
    red = robot.scene.objects(["cube_red"])[0]
    rx, ry, rz = red.position
    size = red.size[2]
    robot.scene.clear(red.id, margin=0.05)
    down = [math.pi, 0.0, 0.0]

    robot.gripper.open()
    robot.arm.move_pose([rx, ry, rz + 0.12], down)
    robot.arm.move_linear(dz=-0.12)  # fingertips at the cube's centre height
    robot.gripper.close()
    robot.arm.move_linear(dz=0.15)
    robot.log("lifted the red cube")

    blue = robot.scene.objects(["cube_blue"])[0]
    bx, by, bz = blue.position
    robot.scene.clear(blue.id, margin=0.05)
    seat = bz + size  # where the red cube's centre sits when it rests on the blue cube
    robot.arm.move_pose([bx, by, seat + 0.10], down)
    robot.arm.move_linear(dz=-0.08)  # red cube's bottom 2 cm above the blue cube's top
    robot.arm.move_linear(dz=-0.015)  # 0.5 cm above the seat: let it drop the last bit, never press
    robot.gripper.open()
    robot.arm.move_linear(dz=0.12)
    robot.log("released on the blue cube")
