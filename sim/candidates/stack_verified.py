import math


def run(robot, task):
    """Build blue/red/yellow with measured grasp verification between placements.

    Sim-only candidate. A failed lift aborts before the next cube is touched.
    The lab must independently judge the final stack and its stability.
    """
    down = [math.pi, 0.0, 0.0]
    # Simulation geometry, copied from dimos METAL_HEURISTIC_AIM:
    # planning tip is gripper_base; pad centre is 0.078 m ahead of it.
    pad_offset = 0.078
    target_name = "cube_blue"
    for source_name in ["cube_red", "cube_yellow"]:
        source = robot.scene.objects([source_name])[0]
        target = robot.scene.objects([target_name])[0]
        sx, sy, sz = source.position
        tx, ty, tz = target.position
        robot.scene.clear(source.id, margin=0.05)
        robot.scene.clear(target.id, margin=0.05)
        robot.gripper.open()
        robot.arm.move_pose([sx, sy, sz + pad_offset + 0.10], down)
        robot.arm.move_linear(dz=-0.10)
        robot.gripper.close()
        robot.arm.move_linear(dz=0.15)
        lifted = robot.scene.objects([source_name])[0]
        if lifted.position[2] - sz < 0.08:
            raise RuntimeError("failed lift: " + source_name)
        robot.log("verified lift: " + source_name)
        # Use the measured offset between the tool and the held cube rather
        # than assuming the fingertips and cube centre coincide.
        tool = robot.arm.pose()
        offset = [tool.position[i] - lifted.position[i] for i in range(3)]
        destination_z = tz + (target.size[2] + source.size[2]) / 2
        robot.arm.move_pose(
            [tx + offset[0], ty + offset[1], destination_z + offset[2] + 0.10], down
        )
        robot.arm.move_linear(dz=-0.10)
        robot.gripper.open()
        robot.arm.move_linear(dz=0.12)
        placed = robot.scene.objects([source_name])[0]
        if (math.hypot(placed.position[0] - tx, placed.position[1] - ty) > 0.015
                or abs(placed.position[2] - destination_z) > 0.012):
            raise RuntimeError("failed placement: " + source_name)
        robot.log("verified placement: " + source_name)
        target_name = source_name
