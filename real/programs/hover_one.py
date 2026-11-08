import math


def run(robot, task):
    """Hover check: one planned move that puts the finger pads 10 cm above a measured object's centre, jaw pointing
    down, then holds 5 s so the operator can see whether the jaw is centred over the object. Task text names the
    object ("hover over cube_red") and optionally the pad height in metres ("... at 0.15"). Corrects the gravity sag
    with up to two re-aims (tower v24's 0.7 gain) and logs the residual error. Nothing is grasped."""
    pad = 0.078
    words = [w.strip(".,;:'\"").lower() for w in task.text.split()]
    name = next((w for w in words if "_" in w), "cube_red")
    height = 0.10
    for w in words:
        try:
            height = float(w)
        except ValueError:
            pass
    obj = robot.scene.objects([name])[0]
    x, y, z = obj.position
    yaw = obj.yaw - (math.pi / 2) * round(obj.yaw / (math.pi / 2))
    target = (x, y, z + pad + height)
    robot.log("hover over %s at (%.3f, %.3f, %.3f): mount at z %.3f" % (name, x, y, z, target[2]))
    cx, cy, cz = target
    for attempt in range(3):
        robot.arm.move_pose([cx, cy, cz], [math.pi, 0.0, yaw])
        p = robot.arm.pose().position
        ex, ey, ez = target[0] - p[0], target[1] - p[1], target[2] - p[2]
        robot.log("attempt %d: mount (%.3f, %.3f, %.3f), off (%.1f, %.1f, %.1f) cm" % (attempt, p[0], p[1], p[2], ex * 100, ey * 100, ez * 100))
        if math.hypot(ex, ey) <= 0.004 and abs(ez) <= 0.006:
            break
        cx, cy, cz = cx + 0.7 * ex, cy + 0.7 * ey, cz + 0.7 * ez
    robot.observe()
    for _ in range(150):  # ~5 s of pose reads: look at the jaw from the side and from the front
        robot.arm.pose()
    robot.observe()
