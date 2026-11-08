import math


def run(robot, task):
    """Cable cycle v2: v1's re-plug done with absolute re-aims, because the traces showed every straight move
    with the plug in the jaw undershooting by the sag. v1 (0/5): the 6 cm straight lift raised the plug only
    4.2-4.9 cm (once 3.2), the straight xy corrections of 1-1.5 cm moved the plug 0-5 mm (a straight move plans
    from the measured pose, so it re-commands the sag and shifts the settled pose by the request minus the sag)
    and the straight descents stopped the housing 2.0-2.6 cm above its seat with the blade still above the mouth,
    which the program took for a blade on the rim; the release then dropped the plug onto the socket, tilted.
    v2 keeps v1's grasp (cable_v1's, 15/15 trials) with an 8 cm lift, then drives the plug with move_pose on a
    command it carries along: command += 0.7 x (socket axis - measured plug xy), the lifted height first, until the
    plug is within 1.5 mm of the axis; then the insertion, the command z lowered towards the seat in steps of at
    most 1.2 cm with the xy correction continued, the plug's own z measured after every step; a step that lowers
    the plug under half of what it asked while the housing is still 4 mm above the seat is a jammed blade: the
    command rises 2 cm, the plug is re-aligned and the insertion retried (twice). The jaw opens once the housing
    is within 3 mm of its seat, then the arm rises on a planned move.
    Expected failure: the xy sag changing during the 2.5 cm insertion and pressing the blade against the slot wall
    (1 mm clearance) so the plug tilts in the pads; IK failures at the end of the trial."""
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

    seated = False
    for attempt in range(3):
        ex, ey, up = align("aligned at the lifted height")
        if math.hypot(ex, ey) > 0.004:
            robot.log("not aligned, trying once more from 2 cm higher")
            c[2] += 0.02
            command(c, yaw)
            ex, ey, up = align("re-aligned")
        # insertion: lower the command toward the seat, the plug's own height as feedback
        jammed = False
        for step in range(8):
            if up <= 0.003:
                seated = True
                break
            drop = min(0.012, up)
            c[2] -= drop
            c[0] += gain * ex
            c[1] += gain * ey
            command(c, yaw)
            dwell(5)
            ex2, ey2, up2 = plug_state()
            robot.log("insert step %d: asked %.1f cm down, plug went %.1f cm; (%.1f, %.1f) mm off axis, %.1f mm above the seat" % (
                step, drop * 100, (up - up2) * 100, ex2 * 1000, ey2 * 1000, up2 * 1000))
            if up2 > 0.004 and (up - up2) < 0.5 * drop and up < 0.030:
                jammed = True
                ex, ey, up = ex2, ey2, up2
                break
            ex, ey, up = ex2, ey2, up2
        if seated or (up <= 0.003):
            seated = True
            break
        if jammed:
            robot.log("the blade is jammed %.1f mm above the seat, lifting 2 cm to realign" % (up * 1000))
        else:
            robot.log("the plug stopped %.1f mm above the seat, lifting 2 cm to realign" % (up * 1000))
        c[2] += 0.02
        command(c, yaw)
    if seated:
        robot.gripper.open()
        dwell(10)
        done = find(plug.name)
        robot.log("released: housing %.1f mm above the seat, %.1f mm off axis" % ((done.position[2] - pz) * 1000, math.hypot(done.position[0] - px, done.position[1] - py) * 1000))
    else:
        robot.log("could not seat %s; releasing it over the socket" % plug.name)
        robot.gripper.open()
        dwell(10)
    c[2] += 0.08
    command(c, yaw)
    go_high()
