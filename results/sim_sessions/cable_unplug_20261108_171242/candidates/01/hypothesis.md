Cable v1: unplug every plug from the cable set, nearest first. Each plug stands in a vertical socket of the
    hub, so an unplug is a straight lift: the jaw comes down over the plug's housing (reported by scene.objects as
    a 1.8 x 1.2 x 4 cm box standing on the hub), squeezes it across the strip (tool yaw = the plug's yaw, since the
    jaw opens along the tool's y axis and the fingers are 4 cm wide along x), and rises 8 cm. The pads are aimed
    7.8 cm ahead of gripper_base and 5 mm above the housing centre so the fingertips stay about 1 cm above the hub.
    Re-aims with a 0.7 gain cancel the sag (tower v5 idiom), the sag learned at the hover is fed into the grasp,
    every gripper command is followed by a dwell, straight moves fall back to planned ones, travel is at 0.25 m
    with step-down fallbacks. The hub and the plug are cleared from the obstacle map before the descent, th