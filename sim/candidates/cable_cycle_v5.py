import math


def run(robot, task):
    """Cable cycle v5: put the blade tip in the funnel, let go completely, then press the plug home with the
    closed jaw. v3 (1/5) and v4 (0/5) on the Mac showed that a planned move asked to go 4 mm lands the hanging
    plug anywhere from 10 mm up to 18 mm down and 5-10 mm sideways, so millimetre stepping with a held plug is
    noise, and that v4's one-third jaw opening did not free the plug at all (it stayed to the 0.1 mm through the
    dwell in every trial), while a push with the jaw closed on the housing left it 13-21 mm high, tilted. v5 keeps
    the one planned move that lands the blade tip in the funnel (checked within 6 mm, retried from above), then
    opens the jaw fully so the plug is free to slide down the funnel (a free plug seats from 6 mm off in plain
    MuJoCo), reads it, and if the housing is still above its seat closes the empty jaw and presses the plug's cable
    stub down with the closed fingertips (9.2 cm ahead of the mount): a re-aimed move to 2 cm above the stub, then
    a command 8 mm past the seated stub top, repeated 1 cm lower while the plug stays high (three presses). The
    arm then rises.
    Expected failure: the free plug tipping over in the funnel before the press, or the press landing beside the
    3 mm stub and on the housing's edge, tilting it."""
    down_roll = math.pi
    pad = 0.078
    hover = 0.08
    travel_z = 0.25
    gain = 0.7
    grip_up = 0.005
    lift = 0.08

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

    def command(c, yaw):
        try:
            robot.arm.move_pose(list(c), [down_roll, 0.0, yaw])
            return True
        except Exception as error:
            robot.log("re-aim to (%.3f, %.3f, %.3f) refused (%s)" % (c[0], c[1], c[2], str(error)[:40]))
            return False

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

    # --- unplug (cable_v1) ---
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

    # --- plug back in, on an absolute command carried along ---
    # the command that put the pads at the grasp was target + sag; start from it at the lifted height
    tool = robot.arm.pose().position
    c = [px + sag[0], py + sag[1], tool[2] + sag[2]]
    in_hand = (held.position[0] - tool[0], held.position[1] - tool[1])  # where the plug hangs relative to the tool
    robot.log("plug hangs (%.1f, %.1f) mm from the tool axis" % (in_hand[0] * 1000, in_hand[1] * 1000))

    def plug_state():
        h = find(plug.name)
        return px - h.position[0], py - h.position[1], h.position[2] - pz  # xy error of the plug, housing above seat

    def align(label, tries=6, tol=0.0015):
        ex, ey, up = plug_state()
        for attempt in range(tries):
            if math.hypot(ex, ey) <= tol:
                break
            c[0] += gain * ex
            c[1] += gain * ey
            if not command(c, yaw):
                break
            ex, ey, up = plug_state()
        robot.log("%s: plug (%.1f, %.1f) mm off the socket axis, housing %.1f cm above the seat, command z %.3f" % (label, ex * 1000, ey * 1000, up * 100, c[2]))
        return ex, ey, up

    def lost(up):
        # the plug has left the pads when it hangs far lower than the tool would hold it
        tool_z = robot.arm.pose().position[2]
        return (pz + up) < tool_z - pad - grip_up - 0.02

    def approach(label, tries=6):
        # align the hanging plug with its blade 1 cm above the mouth: xy within 1.5 mm, z within 4 mm
        ex, ey, up = plug_state()
        for attempt in range(tries):
            ez = 0.035 - up
            if math.hypot(ex, ey) <= 0.0015 and abs(ez) <= 0.004:
                break
            c[0] += gain * ex
            c[1] += gain * ey
            c[2] += gain * ez
            if not command(c, yaw):
                break
            ex, ey, up = plug_state()
        robot.log("%s: plug (%.1f, %.1f) mm off the socket axis, housing %.1f mm above the seat" % (label, ex * 1000, ey * 1000, up * 1000))
        return ex, ey, up

    tip = 0.092  # closed fingertips ahead of the mount
    stub_top = pz + 0.035  # the seated plug's cable stub top (housing top + 1.5 cm)
    seated = False
    dropped = False
    for attempt in range(3):
        ex, ey, up = approach("over the mouth" if attempt == 0 else "back over the mouth")
        if math.hypot(ex, ey) > 0.004:
            robot.log("not aligned within 4 mm, trying again")
            continue
        c[2] -= (up - 0.021)  # blade tip 4 mm into the funnel
        c[0] += gain * ex
        c[1] += gain * ey
        command(c, yaw)
        dwell(3)
        ex, ey, up = plug_state()
        robot.log("blade at the funnel: (%.1f, %.1f) mm off axis, housing %.1f mm above the seat" % (ex * 1000, ey * 1000, up * 1000))
        if lost(up):
            robot.log("the plug left the pads")
            dropped = True
            break
        if math.hypot(ex, ey) > 0.006 or up > 0.030:
            robot.log("not in the funnel, lifting to re-align")
            c[2] += 0.035 - up
            command(c, yaw)
            continue
        robot.gripper.open()
        dwell(15)
        dropped = True
        ex, ey, up = plug_state()
        robot.log("after letting go: (%.1f, %.1f) mm off axis, housing %.1f mm above the seat" % (ex * 1000, ey * 1000, up * 1000))
        break
    if dropped:
        ex, ey, up = plug_state()
        if up <= 0.003:
            seated = True
        else:
            # press the stub down with the closed fingertips
            robot.gripper.close()
            dwell(10)
            robot.scene.clear(find(plug.name).id, margin=0.05)
            above = stub_top + up + 0.02 + tip  # fingertips 2 cm above the stub as it stands now
            p, sag2 = reach(px, py, above, yaw, "over the stub", tries=4, xy_tol=0.005, z_tol=0.006, offset=(sag[0], sag[1], sag[2]))
            for press in range(3):
                target = stub_top - 0.008 - 0.010 * press + tip
                command([px + sag2[0], py + sag2[1], target + sag2[2]], yaw)
                dwell(5)
                ex, ey, up = plug_state()
                robot.log("press %d: housing %.1f mm above the seat, (%.1f, %.1f) mm off axis" % (press, up * 1000, ex * 1000, ey * 1000))
                if up <= 0.003:
                    seated = True
                    break
            command([px + sag2[0], py + sag2[1], above + sag2[2]], yaw)
    if seated:
        done = find(plug.name)
        robot.log("seated: housing %.1f mm above the seat, %.1f mm off axis" % ((done.position[2] - pz) * 1000, math.hypot(done.position[0] - px, done.position[1] - py) * 1000))
    else:
        robot.log("could not seat %s (%.1f mm up, (%.1f, %.1f) mm off)" % (plug.name, up * 1000, ex * 1000, ey * 1000))
        robot.gripper.open()
        dwell(10)
    c[2] += 0.08
    command(c, yaw)
    go_high()
