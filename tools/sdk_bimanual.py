"""Dream Labs ControlLoop with the station's verified paired Metal parking."""
import argparse
import asyncio
import base64
import dataclasses
import json
import logging
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np

CHECKPOINT = "dreamlabs/dream-1-beta"
RIGHT_MULTITASK = "dream-1-ft:2d43713b-right-arm-v1"
RIGHT_UNPLUG = "dream-1-ft:2d43713b-right-unplug-v1"
UNPLUG_TASK = "Using the right arm, unplug all the cables from the cable set."
TASK = ("Use both arms together to unplug the cables from the power strip. "
        "Approach each plug's rigid upper housing from its exposed side. Keep the gripper's approach "
        "axis horizontal and parallel to the tabletop, 90 degrees from vertical. Close the jaws around "
        "the upper plastic housing opposite the end inserted into the socket. Once the plug is firmly "
        "grasped, pull it straight out along its insertion axis without twisting. Do not pull on the wire. "
        "Keep the arms clear of each other.")


class RequestBudget:
    """Cap inference attempts while leaving the SDK's pacing and playback intact."""

    def __init__(self, client, limit, output):
        self.client, self.limit, self.output = client, limit, output
        self.calls = self.succeeded = 0
        self.events = []
        self._lock = threading.Lock()

    def __getattr__(self, name):
        return getattr(self.client, name)

    def refill_due(self, executed, remaining):
        return self.calls < self.limit and self.client.refill_due(executed, remaining)

    def get_action(self, **kwargs):
        with self._lock:
            if self.calls >= self.limit:
                raise RuntimeError("Inference request budget exhausted")
            self.calls += 1
            number = self.calls
        event = {"call": number}
        started = time.monotonic()
        logging.info("Inference API call %d/%d", number, self.limit)
        try:
            result = self.client.get_action(**kwargs)
            self.succeeded += 1
            event["status"] = "success"
            return result
        except Exception as error:
            event.update(status="error", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            event["elapsed_ms"] = (time.monotonic() - started) * 1000
            self.events.append(event)
            (self.output / "api-calls.json").write_text(json.dumps(self.events, indent=2) + "\n")


class HoldLeftClient:
    """Run the right policy targets while holding the left arm at its captured pose."""

    def __init__(self, client, joints, grip, horizon):
        self.client = client
        row = np.concatenate([np.asarray(joints, dtype=float).copy(), [float(grip)]])
        if not np.isfinite(row).all() or not 0 <= grip <= 1:
            raise ValueError("Invalid left hold pose")
        self.left_rows = np.repeat(row[None], horizon, axis=0)

    def __getattr__(self, name):
        return getattr(self.client, name)

    def get_action(self, **kwargs):
        reply = self.client.get_action(**kwargs)
        return {**reply, "q_l_chunk": self.left_rows.copy()}


class ShadowArm:
    """SDK follower interface backed by paired read-only snapshots; writes stay in memory."""
    dof = 6
    drops_when_limp = True

    def __init__(self, side, shared):
        self.side, self.shared = side, shared
        self.last = None

    def read(self):
        from robots.types import ArmState
        joints, grips = self.shared[0]
        return ArmState(joints[self.side].copy(), np.zeros(6), grips[self.side],
                        None if self.last is None else self.last.positions_rad.copy(),
                        None if self.last is None else self.last.gripper)

    def write(self, command):
        self.last = command

    def alarm(self):
        return self.shared[1]

    def is_alive(self):
        return True

    def hold(self):
        pass

    def disengage(self):
        raise RuntimeError("Read-only shadow must never disengage motors")


def make_app(output, task=TASK):
    from dream_hackathon import config as kit_config
    from robots import config
    from embodiments.metal.teleop import validate_channels
    station = kit_config.load(Path.home() / "physical-ai-hacks/configs/nuck4-subtasks.yaml")
    validate_channels(station)
    source = config.load(str(Path.home() / ".dreamlabs/station.yaml"))
    app = dataclasses.replace(
        source, checkpoint=CHECKPOINT, task=task, control_hz=20, ik_solve="tick",
        cameras=[config.CameraConfig(name=c.role, device=c.device, width=c.width,
                                     height=c.height, fps=c.fps) for c in station.cameras],
        policy_image_size=(426, 240),
        first_chunk_timeout_s=20, action_timeout_s=3,
        ramp_s=2.0, ramp_max_rad_s=0.2, max_ee_speed_m_s=0.08,
        max_gripper_step=0.05, command_hz=250, command_interp_max_s=0.1,
        trace_path=str(output / "trace.csv"), inference_log_path=str(output / "inference.jsonl"))
    return station, app


def open_single_client(args, policy):
    """An explicit fine-tune must never fall back to the base-model client."""
    if args.right_checkpoint:
        from fine_tuned_client import RightFineTuneClient
        return RightFineTuneClient.open(args.right_checkpoint, args.training_archive, args.output)
    from robots import run
    return run.open_client(policy)


async def attempt(args, page):
    from robots import arms_lock, control, run
    from embodiments.metal.teleop import MetalArms
    station, app = make_app(args.output, args.task)
    monitor = None
    if args.stack_three:
        from stack_completion import StackMonitor
        monitor = StackMonitor(args.output / "completion.jsonl")
    if args.right_checkpoint:
        from fine_tuned_client import training_metadata
        info, _ = training_metadata(args.training_archive)
        hz = float(info["fps"])
        if hz != 30:
            raise ValueError("Fine-tuned playback requires the verified 30 Hz right-arm schema")
        app = dataclasses.replace(app, control_hz=hz, sync_above_ms=600,
                                  max_gripper_step=0.05 * 20 / hz, policy_image_size=(640, 360))
    loop_args = run.loop_settings(app)
    settings = {k: v for k, v in loop_args.items() if k != "wire"}
    settings.update(checkpoint=CHECKPOINT, task=app.task, live=args.live, seconds=args.seconds,
                    active_policy_arms=["right"] if args.right_only else ["left", "right"],
                    max_api_calls=args.max_api_calls,
                    loop="robots.control.ControlLoop", sdk_pacing="client.refill_due",
                    policy=run.policy_config(app, CHECKPOINT),
                    cameras={c.name: c.device for c in app.cameras})
    if monitor is not None:
        from stack_completion import SETTLE_S
        settings["completion"] = {"endpoint": "POST /v1/decisions", "model": "gpt-6-luna",
            "questions": monitor.questions, "placement_threshold": 0.90,
            "release_threshold": 0.95, "settle_s": SETTLE_S,
            "on_candidate": "stop SDK execution and hold; confirm on a new frame",
            "on_end": "hold until explicit operator STOP/park; never auto-resume"}
    if args.separate_arms:
        from dual_client import DualClient, PROMPTS
        settings.update(independent_api_streams=2, arm_prompts=PROMPTS,
                        playback="paired SDK executor; each stream supplies only its assigned arm")
        if args.right_checkpoint:
            settings["arm_prompts"] = {**PROMPTS,
                "right": "Using the right arm, unplug all the cables from the cable set."}
            settings.update(models_by_arm={"left": CHECKPOINT, "right": args.right_checkpoint},
                            native_action_hz={"left": 20, "right": info["fps"]},
                            training_archive=str(args.training_archive))
    elif args.right_checkpoint:
        settings.update(checkpoint=args.right_checkpoint,
                        models_by_arm={"right": args.right_checkpoint},
                        native_action_hz={"right": info["fps"]},
                        training_archive=str(args.training_archive),
                        policy={"checkpoint": args.right_checkpoint, "action_hz": info["fps"],
                                "sync_above_ms": 600,
                                "embodiment_source": "checkpoint metadata; see right-model.json"})
    (args.output / "settings.json").write_text(json.dumps(settings, indent=2, default=str) + "\n")
    lease, arms, client, loop = None, None, None, None
    dual_client = None
    readers = []
    preview_task = poll_task = runner = monitor_task = None
    finish = asyncio.Event()
    monitor_done = asyncio.Event()
    held_at = None
    completion_status = None
    policy_stopped = False
    powered = parked = False
    reason = "time limit"

    async def preview():
        while not finish.is_set():
            for reader in readers:
                frame = reader.frame
                if frame is not None and frame.jpeg:
                    image = base64.b64decode(frame.jpeg)
                    page.frames[reader.name] = image
                    (args.output / f"{reader.name}.jpg").write_bytes(image)
            await asyncio.sleep(0.25)

    try:
        page.status = "Preparing SDK control loop and Metal IK; no motor motion"
        if args.separate_arms:
            wire = loop_args["wire"].for_arms({"left": len(app.follower_l.home),
                                             "right": len(app.follower_r.home)})
            dual_client = await asyncio.to_thread(DualClient.open, settings["policy"], args.output,
                                                  wire, app.action_horizon, args.right_checkpoint,
                                                  args.training_archive)
            client = dual_client
        else:
            client = await asyncio.to_thread(open_single_client, args, settings["policy"])
        if args.max_api_calls is not None:
            client = RequestBudget(client, args.max_api_calls, args.output)
        if not await asyncio.to_thread(client.wait_for_ik, 60):
            raise RuntimeError("Metal IK warmup incomplete")
        kinematics = await asyncio.to_thread(run.make_kinematics, app)
        lease = arms_lock.acquire(Path.home() / ".dreamlabs/arms.lock", "SDK policy execution")
        readers = await asyncio.to_thread(run.make_cameras, app)
        for reader in readers:
            reader.start()
        preview_task = asyncio.create_task(preview())
        deadline = time.monotonic() + 10
        while any(r.frame is None for r in readers):
            if page.stop.is_set():
                reason = "operator stop before enable"
                return
            if time.monotonic() > deadline:
                raise RuntimeError("A required camera has no frame")
            await asyncio.sleep(0.1)
        if page.stop.is_set():
            reason = "operator stop before enable"
            return
        if monitor is not None:
            # Real camera/API preflight before opening either motor connection.
            preflight = await monitor.check(readers)
            if preflight != "continue":
                reason = f"Stack preflight: {preflight}; motors not enabled"
                completion_status = preflight
                return
            if page.stop.is_set():
                reason = "operator stop during Decisions preflight"
                return
        page.status = "Powering both arms" if args.live else "SDK dry run; motors unchanged"
        arms = await asyncio.to_thread(MetalArms, station, energize=args.live)
        powered = args.live
        if args.live:
            joints, grips = await asyncio.to_thread(arms.read)
            for _ in range(30):
                if page.stop.is_set():
                    reason = "operator stop during startup sync"
                    return
                await asyncio.to_thread(arms.command, joints, grips)
                await asyncio.sleep(0.05)
            followers = arms._arms
        else:
            shared = [await asyncio.to_thread(arms.read), None]
            followers = {side: ShadowArm(side, shared) for side in ("left", "right")}

            async def poll():
                try:
                    while not finish.is_set():
                        shared[0] = await asyncio.to_thread(arms.read)
                        await asyncio.sleep(0.05)
                except Exception as error:
                    shared[1] = str(error)
                    page.stop.set()
            poll_task = asyncio.create_task(poll())
        if args.right_only:
            hold_joints, hold_grips = (joints, grips) if args.live else shared[0]
            client = HoldLeftClient(client, hold_joints["left"], hold_grips["left"], app.action_horizon)
            (args.output / "left-hold.json").write_text(json.dumps(
                {"joints_rad": hold_joints["left"].tolist(), "gripper": hold_grips["left"]}, indent=2) + "\n")
        loop = control.ControlLoop(followers, readers, client, task=app.task,
                                   kinematics=kinematics, camera_owner=False,
                                   inference_log_path=app.inference_log_path, **loop_args)
        async with loop:
            runner = asyncio.create_task(loop.run())
            if monitor is not None:
                monitor_task = asyncio.create_task(monitor.watch_candidate(readers, monitor_done))
            first_motion = None
            started = time.monotonic()
            page.status = "Running both arms with SDK pacing" if args.live else "SDK dry run; target commands only"
            if args.live and args.right_only:
                page.status = "Running right arm with SDK pacing; left arm holding"
            try:
                while not runner.done() and not page.stop.is_set():
                    if monitor is not None and monitor.overdue(time.monotonic()):
                        completion_status = "monitor unavailable: decision deadline exceeded"
                        reason = f"Decisions API: {completion_status}"
                        break
                    if monitor_task is not None and monitor_task.done():
                        completion_status = monitor_task.result()
                        reason = f"Decisions API: {completion_status}"
                        break
                    if (args.max_api_calls is not None and client.calls >= args.max_api_calls
                            and loop._refill is None and not loop._buffer):
                        # Let the final interpolated command finish; the budget blocks all refills.
                        await asyncio.sleep(app.command_interp_max_s)
                        reason = ("API call limit completed" if loop.chunk_count == args.max_api_calls
                                  else "API call limit reached; some responses were not executed")
                        break
                    if loop.steps_done and first_motion is None:
                        first_motion = time.monotonic()
                    if first_motion is not None and time.monotonic() - first_motion >= args.seconds:
                        break
                    if time.monotonic() - started > args.seconds + 25:
                        reason = "startup deadline"
                        break
                    await asyncio.sleep(0.02 if monitor is not None else 0.1)
                if page.stop.is_set():
                    reason = "operator stop"
                if runner.done():
                    runner.result()
            finally:
                monitor_done.set()
                from stack_completion import stop_policy
                from embodiments.metal.supervised import hold_pair
                held_at = await stop_policy(loop, runner, followers, hold_pair)
                policy_stopped = True
                if monitor_task is not None:
                    outcome = await monitor_task
                    if outcome != "ended" and completion_status is None:
                        completion_status = outcome
                        if not page.stop.is_set():
                            reason = f"Decisions API: {outcome}"
                await asyncio.to_thread(client.close)
        if monitor is not None and completion_status == "candidate" and not page.stop.is_set():
            page.status = "HOLDING: checking that the released three-block tower stays stacked"
            completion_status = await monitor.confirm_held(readers, held_at)
            reason = f"Decisions API: {completion_status}"
    except Exception as error:
        reason = f"{type(error).__name__}: {error}"
        logging.exception("Attempt ended")
    finally:
        monitor_done.set()
        if loop is not None and not policy_stopped:
            from stack_completion import stop_policy
            from embodiments.metal.supervised import hold_pair
            try:
                held_at = await stop_policy(loop, runner, followers, hold_pair)
            except Exception as error:
                # A bad paired read must not cause a partial new hold target or
                # drop the lease/motor workers. Retain their previous targets.
                reason = f"Paired hold unavailable: {type(error).__name__}: {error}"
                logging.exception(reason)
        if monitor_task is not None:
            await monitor_task  # Finish snapshot/API workers before closing cameras.
        if monitor is not None:
            (args.output / "completion-state.json").write_text(json.dumps({
                "state": completion_status or "incomplete", "reason": reason,
                "held_at": held_at, "powered": powered,
                "policy_stopped": True, "automatic_resume": False}, indent=2) + "\n")
            if powered and not page.stop.is_set():
                from stack_completion import wait_for_park
                await wait_for_park(page, reason)
        finish.set()
        page.stop.set()
        if poll_task is not None:
            await poll_task
        page.status = "Parking and verifying both arms" if powered else "Closing read-only check"
        if arms is not None:
            await asyncio.to_thread(arms.close)
            parked = powered
        if preview_task is not None:
            await preview_task
        if client is not None:
            await asyncio.to_thread(client.close)
        for reader in readers:
            await asyncio.to_thread(reader.stop)
        if lease is not None:
            lease.close()
        result = {"reason": reason, "live": args.live, "powered": powered,
                  "completion": completion_status,
                  "parked_and_disabled": parked, "chunks": loop.chunk_count if loop else 0,
                  "steps": loop.steps_done if loop else 0,
                  "timing": loop.timing.as_dict() if loop else None}
        if isinstance(client, RequestBudget):
            result.update(api_calls=client.calls, successful_api_calls=client.succeeded)
        if dual_client is not None:
            result.update(dual_client.stats())
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        print("RESULT", json.dumps(result), flush=True)
        page.status = "Finished; both arms parked and disabled" if parked else "Finished; motors were not enabled"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--task", default=TASK)
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--max-api-calls", type=int)
    parser.add_argument("--separate-arms", action="store_true")
    parser.add_argument("--right-only", action="store_true")
    parser.add_argument("--stack-three", action="store_true",
                        help="Right-arm three-block tower; Decisions stop-and-hold gate")
    parser.add_argument("--right-checkpoint")
    parser.add_argument("--training-archive", type=Path,
                        help="Matching training export ZIP containing meta/info.json and meta/tasks.jsonl")
    parser.add_argument("--output", required=True, type=Path)
    sys.path.insert(0, str(Path.home() / "Superfast-harness"))
    args = parser.parse_args(argv)
    if args.stack_three:
        if args.separate_arms:
            parser.error("stack-three requires a single right-arm policy")
        args.right_checkpoint = args.right_checkpoint or RIGHT_MULTITASK
        if args.right_checkpoint == RIGHT_UNPLUG:
            parser.error("The unplug-only checkpoint cannot run the stack-three preset")
        from stack_completion import STACK_TASK
        if args.task == TASK:
            args.task = STACK_TASK
        args.right_only = True
    if not 0 < args.seconds <= 120:
        parser.error("seconds must be in (0, 120]")
    if args.max_api_calls is not None and args.max_api_calls < 1:
        parser.error("max-api-calls must be positive")
    if args.right_only and args.separate_arms:
        parser.error("right-only and separate-arms cannot be combined")
    if args.right_checkpoint:
        if not args.right_checkpoint.startswith("dream-1-ft:"):
            parser.error("right-checkpoint must identify a fine-tuned Dream model")
        if not (args.separate_arms or args.right_only):
            parser.error("right-checkpoint requires right-only or separate-arms")
        if args.training_archive is None:
            if args.right_checkpoint == RIGHT_UNPLUG:
                args.training_archive = Path.home() / "hackathon_data/evaluations/right-unplug-v1-20261108/training.zip"
            else:
                parser.error("Provide --training-archive for this checkpoint's matching training export")
        if args.right_only and args.task == TASK:
            args.task = UNPLUG_TASK
    return args


def main():
    args = parse_args()
    args.output.mkdir(exist_ok=False)
    from dotenv import load_dotenv
    from embodiments.metal.supervised import StopPage
    load_dotenv(Path.home() / "Superfast-harness/.env", override=True)
    if args.stack_three:
        load_dotenv(Path.home() / ".dreamlabs/openai.env", override=False)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    page = StopPage(8091)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *unused: page.stop.set())
    try:
        asyncio.run(attempt(args, page))
    finally:
        page.server.shutdown()
        page.server.server_close()


if __name__ == "__main__":
    main()
