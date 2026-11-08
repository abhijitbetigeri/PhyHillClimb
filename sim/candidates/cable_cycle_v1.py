import math


def run(robot, task):
    """Cable cycle v1: unplug the nearest plug and plug it back into the same socket. The grasp and the 6 cm
    straight lift are cable_v1's (5/5 dev, both plugs, 36-38 s). The re-plug uses the scene as the measuring
    tool: the socket axis is where the plug's housing stood before the lift, so after the lift the held plug's
    measured offset from that spot is corrected with short straight moves (relative moves shift the settled pose
    by what they ask, unlike absolute re-aims that fight the sag), to 1 mm. The descent is in two straight steps:
    to 1 cm above the mouth, re-measure and correct, then down to the seat. If the housing is still more than
    4 mm above its seated height the blade is on the rim: rise 2 cm, correct, try again (up to 3 attempts). The
    jaw opens only once the housing is within 4 mm of the seat, then the arm rises clear.
    Expected failure: the held plug tilting in the pads when the blade catches the 1 mm slot clearance, or the
    straight corrections failing to settle under 1 mm."""
    down_roll = math.pi
    pad = 0.078
    hover = 0.08
    travel_z = 0.25
    gain = 0.7
    grip_up = 0.005
    lift = 0.06

    def find(name):
        for attempt in range(3):
            try:
                return robot.scene.objects([name])[0]
            except Exception as error:
                robot.log("%s not in the scene yet (%s), retrying" % (name, error))
        return robot.scene.objects([name])[0]

    def dwell(reads=20):
        for _ in range(reads):
            robot.arm.pose()

    def reach(x, y, z, yaw, label, tries=3, xy_tol=0.004, z_tol=0.004, offset=(0.0, 0.0, 0.0)):
        cx, cy, cz = x + offset[0], y + offset[1], z + offset[2]
        robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
        p = robot.arm.pose().position
        for attempt in range(tries + 1):
            ex, ey, ez = x - p[0], y - p[1], z - p[2]
            if math.hypot(ex, ey) <= xy_tol and abs(ez) <= z_tol:
                break
            if attempt == tries:
                robot.log("%s: still (%.1f, %.1f, %.1f) cm off after %d corrections" % (label, ex * 100, ey * 100, ez * 100, tries))
                break
            cx, cy, cz = cx + gain * ex, cy + gain * ey, cz + gain * ez
            robot.arm.move_pose([cx, cy, cz], [down_roll, 0.0, yaw])
            p = robot.arm.pose().position
        return p, (cx - x, cy - y, cz - z)

    def linear(dx=0.0, dy=0.0, dz=0.0):
        try:
            robot.arm.move_linear(dx=dx, dy=dy, dz=dz)
            return True
        except Exception as error:
            robot.log("straight move (%.1f, %.1f, %.1f) cm failed (%s)" % (dx * 100, dy * 100, dz * 100, str(error)[:50]))
            return False

    def travel_to(x, y, yaw):
        for z in (travel_z, travel_z - 0.03, travel_z - 0.06, travel_z - 0.09):
            try:
                robot.arm.move_pose([x, y, z], [down_roll, 0.0, yaw])
                return z
            except Exception as error:
                robot.log("travel to (%.2f, %.2f) at %.2f m failed (%s), lower" % (x, y, z, str(error)[:60]))
        raise RuntimeError("no reachable travel height over (%.2f, %.2f)" % (x, y))

    def go_high():
        p = robot.arm.pose()
        x, y, z = p.position
        if z >= travel_z - 0.01:
            return
        if linear(dz=travel_z - z):
            return
        for target in (travel_z, travel_z - 0.03, travel_z - 0.06, travel_z - 0.09):
            if target <= z + 0.01:
                return
            try:
                robot.arm.move_pose([x, y, target])
                return
            except Exception:
                continue

    def half_turn(yaw):
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if yaw > math.pi / 2:
            yaw -= math.pi
        elif yaw < -math.pi / 2:
            yaw += math.pi
        return yaw

    def plug_error(name, x, y):
        held = find(name)
        return x - held.position[0], y - held.position[1], held

    def align(name, x, y, label, tol=0.001, tries=3):
        # shift the held plug onto the socket axis with short straight moves
        ex, ey, held = plug_error(name, x, y)
        for attempt in range(tries):
            if math.hypot(ex, ey) <= tol:
                break
            if not linear(dx=ex, dy=ey):
                break
            ex, ey, held = plug_error(name, x, y)
        robot.log("%s: held plug (%.1f, %.1f) mm off the socket axis, housing %.1f cm up" % (label, ex * 1000, ey * 1000, (held.position[2] - pz) * 100))
        return held

    hub = find("hub")
    robot.scene.clear(hub.id, margin=0.03)
    plugs = []
    for name in ("plug_a", "plug_b"):
        try:
            plugs.append(find(name))
        except Exception as error:
            robot.log("no %s in the scene (%s)" % (name, error))
    plugs.sort(key=lambda o: o.position[0])
    plug = plugs[0]
    px, py, pz = plug.position  # the housing centre while seated: the socket axis and the seat height
    yaw = half_turn(plug.yaw)
    robot.log("hub at (%.3f, %.3f) yaw %.2f; cycling %s at (%.3f, %.3f, %.3f)" % (hub.position[0], hub.position[1], hub.yaw, plug.name, px, py, pz))
    grasp_z = pz + grip_up + pad

    # --- unplug ---
    robot.scene.clear(plug.id, margin=0.04)
    robot.gripper.open()
    go_high()
    travel_to(px, py, yaw)
    linear(dz=(grasp_z + hover) - robot.arm.pose().position[2])
    p, sag = reach(px, py, grasp_z + hover, yaw, "hover")
    at_close, sag = reach(px, py, grasp_z, yaw, "grasp", offset=sag, xy_tol=0.003)
    robot.gripper.close()
    dwell()
    linear(dz=lift)
    held = find(plug.name)
    rise = held.position[2] - pz
    if rise < 0.04:
        robot.log("%s rose only %.1f cm, regrasping 5 mm deeper" % (plug.name, rise * 100))
        robot.gripper.open()
        dwell(10)
        at_close, sag = reach(px, py, grasp_z - 0.005, yaw, "regrasp", offset=sag, xy_tol=0.003)
        robot.gripper.close()
        dwell()
        linear(dz=lift + 0.005)
        held = find(plug.name)
        rise = held.position[2] - pz
    robot.log("%s lifted %.1f cm (tool (%.1f, %.1f) mm off the housing when closing)" % (plug.name, rise * 100, (px - at_close[0]) * 1000, (py - at_close[1]) * 1000))
    robot.scene.clear(held.id, margin=0.06)
    if rise < 0.04:
        robot.log("not unplugged, giving up")
        go_high()
        return

    # --- plug back in ---
    seated = False
    for attempt in range(3):
        held = align(plug.name, px, py, "aligned over the socket")
        up = held.position[2] - pz  # how far the housing is above its seat
        if up > 0.012:
            linear(dz=-(up - 0.010))  # blade bottom 1 cm above the mouth
            held = align(plug.name, px, py, "at the mouth")
            up = held.position[2] - pz
        linear(dz=-up)
        dwell(5)
        held = find(plug.name)
        up = held.position[2] - pz
        off = math.hypot(held.position[0] - px, held.position[1] - py)
        robot.log("after the descent: housing %.1f mm above the seat, %.1f mm off axis" % (up * 1000, off * 1000))
        if up <= 0.004:
            seated = True
            break
        robot.log("the blade is on the rim, lifting 2 cm to try again")
        linear(dz=0.02 + max(0.0, 0.012 - up))
    if seated:
        robot.gripper.open()
        dwell(10)
        linear(dz=0.06)
        done = find(plug.name)
        robot.log("released: housing %.1f mm above the seat, %.1f mm off axis" % ((done.position[2] - pz) * 1000, math.hypot(done.position[0] - px, done.position[1] - py) * 1000))
    else:
        robot.log("could not seat %s; holding it over the socket and releasing" % plug.name)
        robot.gripper.open()
        dwell(10)
        linear(dz=0.06)
    go_high()
