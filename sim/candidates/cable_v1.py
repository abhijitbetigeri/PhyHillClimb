import math


def run(robot, task):
    """Cable v1: unplug every plug from the cable set, nearest first. Each plug stands in a vertical socket of the
    hub, so an unplug is a straight lift: the jaw comes down over the plug's housing (reported by scene.objects as
    a 1.8 x 1.2 x 4 cm box standing on the hub), squeezes it across the strip (tool yaw = the plug's yaw, since the
    jaw opens along the tool's y axis and the fingers are 4 cm wide along x), and rises 8 cm. The pads are aimed
    7.8 cm ahead of gripper_base and 5 mm above the housing centre so the fingertips stay about 1 cm above the hub.
    Re-aims with a 0.7 gain cancel the sag (tower v5 idiom), the sag learned at the hover is fed into the grasp,
    every gripper command is followed by a dwell, straight moves fall back to planned ones, travel is at 0.25 m
    with step-down fallbacks. The hub and the plug are cleared from the obstacle map before the descent, the lifted
    plug again after the lift. A plug that rose under 4 cm is regrasped 5 mm deeper once. The unplugged cable is
    carried 10 cm across the strip (away from the hub, on the side with more bench) and released 1.5 cm above the
    bench so the jaw is free for the next plug.
    Expected failure: the first planned descent landing beside the housing (sag of 2-4 cm) so the pads close on
    nothing; the open fingers brushing the other plug."""
    down_roll = math.pi
    pad = 0.078  # finger-pad centre ahead of gripper_base, the frame pose()/move_pose address
    hover = 0.08
    travel_z = 0.25
    gain = 0.7
    grip_up = 0.005  # pads this far above the housing centre: fingertips ~1.2 cm above the hub top
    lift = 0.08
    drop_side = 0.10  # where an unplugged cable goes: this far across the strip

    def find(name):
        for attempt in range(3):
            try:
                return robot.scene.objects([name])[0]
            except Exception as error:
                robot.log("%s not in the scene yet (%s), retrying" % (name, error))
        return robot.scene.objects([name])[0]

    def dwell(reads=20):
        for _ in range(reads):  # ~30 ms per read: lets the jaw finish moving (gripper calls do not wait)
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
        except Exception as error:
            p = robot.arm.pose()
            robot.log("straight move failed (%s), planning it" % str(error)[:50])
            robot.arm.move_pose([p.position[0] + dx, p.position[1] + dy, p.position[2] + dz])

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
        try:
            robot.arm.move_linear(dz=travel_z - z)
            return
        except Exception as error:
            robot.log("straight rise failed (%s), planning a lower one" % str(error)[:40])
        for target in (travel_z, travel_z - 0.03, travel_z - 0.06, travel_z - 0.09):
            if target <= z + 0.01:
                return
            try:
                robot.arm.move_pose([x, y, target])
                return
            except Exception:
                continue
        robot.log("could not rise above %.2f m here, carrying on" % z)

    def half_turn(yaw):
        # the jaw is symmetric: keep the wrist within a quarter turn of the arm's forward direction
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if yaw > math.pi / 2:
            yaw -= math.pi
        elif yaw < -math.pi / 2:
            yaw += math.pi
        return yaw

    hub = find("hub")
    robot.scene.clear(hub.id, margin=0.03)  # the jaw works 1 cm above the hub: it must not count as an obstacle
    plugs = []
    for name in ("plug_a", "plug_b"):
        try:
            plugs.append(find(name))
        except Exception as error:
            robot.log("no %s in the scene (%s)" % (name, error))
    plugs.sort(key=lambda o: o.position[0])  # the plug nearest the arm first
    robot.log("hub at (%.3f, %.3f) yaw %.2f; plugs: %s" % (hub.position[0], hub.position[1], hub.yaw,
                                                          ", ".join("%s (%.3f, %.3f, %.3f)" % (o.name, o.position[0], o.position[1], o.position[2]) for o in plugs)))
    side = 1.0 if hub.position[1] <= 0.0 else -1.0  # drop unplugged cables on the side of the strip with more bench
    unplugged = 0
    for plug in plugs:
        px, py, pz = plug.position  # housing centre, 5.5 cm above the bench when seated
        yaw = half_turn(plug.yaw)
        grasp_z = pz + grip_up + pad
        robot.scene.clear(plug.id, margin=0.04)
        robot.gripper.open()
        go_high()
        travel_to(px, py, yaw)
        linear(dz=(grasp_z + hover) - robot.arm.pose().position[2])
        p, sag = reach(px, py, grasp_z + hover, yaw, "hover over %s" % plug.name)
        at_close, sag = reach(px, py, grasp_z, yaw, "grasp %s" % plug.name, offset=sag, xy_tol=0.003)
        robot.gripper.close()
        dwell()
        linear(dz=lift)
        held = find(plug.name)
        rise = held.position[2] - pz
        if rise < 0.04:
            robot.log("%s rose only %.1f cm, regrasping 5 mm deeper" % (plug.name, rise * 100))
            robot.gripper.open()
            dwell(10)
            at_close, sag = reach(px, py, grasp_z - 0.005, yaw, "regrasp %s" % plug.name, offset=sag, xy_tol=0.003)
            robot.gripper.close()
            dwell()
            linear(dz=lift + 0.005)
            held = find(plug.name)
            rise = held.position[2] - pz
        robot.log("%s lifted %.1f cm (tool at (%.3f, %.3f) when closing, %.1f/%.1f mm off the housing)" % (
            plug.name, rise * 100, at_close[0], at_close[1], (px - at_close[0]) * 1000, (py - at_close[1]) * 1000))
        if rise >= 0.04:
            unplugged += 1
        # set the cable down across the strip, clear of the hub and the other plug
        robot.scene.clear(held.id, margin=0.06)  # the lifted plug is an obstacle again away from its socket
        dx, dy = -drop_side * math.sin(yaw) * side, drop_side * math.cos(yaw) * side
        linear(dx=dx, dy=dy)
        here = robot.arm.pose().position
        drop_z = pad + grip_up + 0.0325 + 0.0125 + 0.015  # blade bottom 1.5 cm above the bench
        if here[2] > drop_z:
            linear(dz=drop_z - here[2])
        robot.gripper.open()
        dwell(10)
        linear(dz=0.06)
        robot.log("%s set down at about (%.2f, %.2f)" % (plug.name, here[0], here[1]))
    robot.log("unplugged %d of %d plugs" % (unplugged, len(plugs)))
    go_high()
