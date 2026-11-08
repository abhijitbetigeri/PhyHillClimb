import math

def run(robot, task):
    """Stack in place on yellow (cube centre), pick each cube top-down with sag-corrected aiming.
    Expect failures from sag at far reach or release disturbing the stack."""
    Q = [math.pi, 0.0, 0.0]
    def aim(x, y, z):
        tx, ty, tz = x, y, z
        for i in range(3):
            try:
                robot.arm.move_pose([tx, ty, tz], Q)
            except Exception:
                if i == 0:
                    raise
                break
            p = robot.arm.pose().position
            tx += 0.7 * (x - p[0]); ty += 0.7 * (y - p[1]); tz += 0.7 * (z - p[2])
    def wait():
        for i in range(30):
            robot.arm.pose()
    def obj(n):
        return [o for o in robot.scene.objects(["cube_"+n]) if n in o.name][0]
    y0 = obj("yellow")
    bx, by = y0.position[0], y0.position[1]
    robot.gripper.open()
    level = 1
    for name in ["red", "blue"]:
        c = obj(name)
        x, y, z = c.position
        robot.scene.clear(c.id)
        robot.gripper.open(); wait()
        aim(x, y, z + 0.078 + 0.10)
        robot.arm.move_linear(dz=-0.10)
        p = robot.arm.pose().position
        err = (z + 0.078) - p[2]
        if err > 0.005:
            robot.arm.move_linear(dz=err)
        robot.gripper.close(); wait(); wait()
        robot.arm.move_linear(dz=0.07)
        zt = 0.025 + 0.05 * level + 0.078 + 0.012
        for o in robot.scene.objects(["cube_blue","cube_red","cube_yellow"]):
            robot.scene.clear(o.id, 0.05)
        robot.scene.clear(y0.id, 0.05)
        aim(bx, by, zt + 0.04)
        robot.arm.move_linear(dz=-0.04)
        robot.gripper.open(); wait(); wait()
        robot.arm.move_linear(dz=0.05)
        level += 1
    p = robot.arm.pose().position
    try:
        robot.arm.move_linear(dz=0.05)
    except Exception:
        pass
    try:
        robot.arm.move_pose([0.2, 0.0, p[2] + 0.15], Q)
    except Exception:
        robot.arm.move_pose([0.2, 0.0, p[2] + 0.05], Q)
