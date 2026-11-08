import math


def run(robot, task):
    """One cube pick and put-back, tower v24's grasp: pads aimed 7.8 cm ahead of the mount at the cube's centre,
    re-aims with a 0.7 gain against sag, the sag learned at the hover fed into the grasp, close, dwell, lift 10 cm,
    hold 3 s (does it stay in the jaw?), then set it back down 3 mm above where it was, open fully, wait, rise.
    Task text names the cube ("pick cube_red"). The scene's grasp belief is logged after the lift."""
    down_roll = math.pi
    pad = 0.078
    hover = 0.10
    gain = 0.7
    words = [w.strip(".,;:'\"").lower() for w in task.text.split()]
    name = next((w for w in words if "_" in w), "cube_red")

    def dwell(reads):
        for _ in range(reads):
            robot.arm.pose()

    def reach(x, y, z, yaw, label, tries=3, offset=(0.0, 0.0, 0.0)):
        cx, cy, cz = x + offset[0], y + offset[1], z + offset[2]
        robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
        p = robot.arm.pose().position
        for attempt in range(tries):
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= 0.004 and abs(ez) <= 0.004:
                break
            cx, cy, cz = cx + gain * ex, cy + gain * ey, cz + gain * ez
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
            p = robot.arm.pose().position
        robot.log("%s: mount (%.3f, %.3f, %.3f), off (%.1f, %.1f, %.1f) cm" % (label, p[0], p[1], p[2], (x - p[0]) * 100, (y - p[1]) * 100, (z - p[2]) * 100))
        return p, (cx - x, cy - y, cz - z)

    obj = robot.scene.objects([name])[0]
    ox, oy, oz = obj.position
    yaw = obj.yaw - (math.pi / 2) * round(obj.yaw / (math.pi / 2))
    grasp_z = oz + pad
    robot.scene.clear(obj.id, margin=0.06)
    robot.gripper.open()
    dwell(20)
    p, sag = reach(ox, oy, grasp_z + hover, yaw, "hover")
    at_close, sag = reach(ox, oy, grasp_z, yaw, "grasp", offset=sag)
    robot.observe()
    robot.gripper.close()
    dwell(20)
    robot.arm.move_linear(dz=hover)
    held = robot.scene.objects([name])[0]
    robot.log("after the lift the scene believes %s at z %.3f (rose %.1f cm); LOOK: is it in the jaw?" % (name, held.position[2], (held.position[2] - oz) * 100))
    robot.observe()
    dwell(90)
    robot.scene.clear(held.id, margin=0.08)
    reach(ox, oy, grasp_z + 0.03, yaw, "over the spot", offset=sag)
    reach(ox, oy, grasp_z + 0.003, yaw, "set-down", offset=sag)
    robot.gripper.open()
    dwell(30)
    robot.arm.move_linear(dz=0.08)
    robot.observe()
