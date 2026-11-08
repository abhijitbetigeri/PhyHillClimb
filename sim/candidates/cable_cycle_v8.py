import math


def run(robot, task):
    """Cable cycle v8: v7 with a second lift when the first one comes up short. v7 scored 3/5 (0.8-1.2 mm off
    axis, full depth, 29-35 s); its dev-02 is the seed that fails the unplug in every version (hub at 0.278 m,
    yaw -0.27): the grasp is within 2 mm of the housing yet the 8 cm straight lift raises the plug only 2.0-3.8 cm,
    and the regrasp 5 mm deeper lifts no more, so the plug is dropped back before the judge's 4 cm. The lift is
    likely short because the straight move near that pose is cut by the planner; v8 lifts again by 5 cm (straight,
    then a planned rise) whenever the plug rose between 1.5 and 4 cm, before deciding to regrasp. The other v7
    failure (dev-01, blade in the slot 9 mm high, presses lifting it instead of seating it) is left as is.
    Expected failure: dev-01's press behaviour; a second lift that the planner also cuts short."""
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
    if 0.015 <= rise < 0.04:
        robot.log("%s rose only %.1f cm in the jaw, lifting again" % (plug.name, rise * 100))
        if not linear(dz=0.05):
            p = robot.arm.pose().position
            command([p[0], p[1], p[2] + 0.07], yaw)
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

    ex, ey, up = approach("over the mouth")
    seated = False
    jams = 0
    stalls = 0
    for step in range(20):
        if up <= 0.003:
            seated = True
            break
        drop = min(0.004, up)
        c[2] -= drop
        c[0] += gain * ex
        c[1] += gain * ey
        command(c, yaw)
        dwell(3)
        ex2, ey2, up2 = plug_state()
        robot.log("step %d: asked %.1f mm down, plug went %.1f mm; (%.1f, %.1f) mm off axis, %.1f mm above the seat" % (
            step, drop * 1000, (up - up2) * 1000, ex2 * 1000, ey2 * 1000, up2 * 1000))
        if lost(up2):
            robot.log("the plug left the pads")
            break
        stalls = stalls + 1 if (up - up2) < 0.25 * drop and up2 > 0.004 else 0
        ex, ey, up = ex2, ey2, up2
        if stalls >= 2:
            jams += 1
            if jams > 3:
                robot.log("jammed three times, giving up")
                break
            robot.log("the blade is caught %.1f mm above the seat, backing off 6 mm to re-align" % (up * 1000))
            c[2] += 0.006
            command(c, yaw)
            ex, ey, up = approach("re-aligned", tries=3) if up > 0.030 else plug_state()
            stalls = 0
    off = math.hypot(ex, ey)
    tip = 0.092  # closed fingertips ahead of the mount
    stub_top = pz + 0.035  # the seated plug's cable stub top
    if seated or (up <= 0.015 and off <= 0.004):
        robot.gripper.open()
        dwell(10)
        ex, ey, up = plug_state()
        robot.log("released: housing %.1f mm above the seat, (%.1f, %.1f) mm off axis" % (up * 1000, ex * 1000, ey * 1000))
        if up > 0.003:
            c[2] += 0.06  # fingers clear of the plug before the jaw closes into a pressing tip
            command(c, yaw)
            robot.gripper.close()
            dwell(10)
            robot.scene.clear(find(plug.name).id, margin=0.05)
            above = stub_top + up + 0.02 + tip
            p, sag2 = reach(px, py, above, yaw, "over the stub", tries=4, xy_tol=0.004, z_tol=0.006, offset=(sag[0], sag[1], sag[2]))
            for press in range(4):
                target = stub_top - 0.008 - 0.010 * press + tip
                command([px + sag2[0], py + sag2[1], target + sag2[2]], yaw)
                dwell(5)
                ex, ey, up = plug_state()
                robot.log("press %d: housing %.1f mm above the seat, (%.1f, %.1f) mm off axis" % (press, up * 1000, ex * 1000, ey * 1000))
                if up <= 0.003:
                    break
            command([px + sag2[0], py + sag2[1], above + sag2[2]], yaw)
        robot.log("final: housing %.1f mm above the seat, %.1f mm off axis" % (up * 1000, math.hypot(ex, ey) * 1000))
    else:
        robot.log("could not seat %s (%.1f mm up, %.1f mm off); releasing it" % (plug.name, up * 1000, off * 1000))
        robot.gripper.open()
        dwell(10)
    c[2] += 0.08
    command(c, yaw)
    go_high()
