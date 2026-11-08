"""Adapt the verified right-arm fine-tune using its actual training tensor order."""
import json
import time
import zipfile

import numpy as np
import dream
from dream.smoothing import SeamSmoother


def training_metadata(archive):
    with zipfile.ZipFile(archive) as z:
        names = [n for n in z.namelist() if n.endswith("meta/info.json")]
        if len(names) != 1:
            raise ValueError("Expected one training metadata file")
        info = json.loads(z.read(names[0]))
        tasks = [json.loads(line)["task"] for line in
                 z.read(names[0].removesuffix("meta/info.json") + "meta/tasks.jsonl").splitlines()]
    return info, tasks


class RightFineTuneClient:
    @classmethod
    def open(cls, checkpoint, archive, output):
        info, tasks = training_metadata(archive)
        client = dream.DreamClient()
        try:
            model = next(m for m in client.get_models() if m.id == checkpoint)
            if not model.servable or model.parts_withheld or not model.embodiment_config:
                raise ValueError("Fine-tune is not available with a complete embodiment")
            body = model.embodiment_config
            adapted = cls(client, body, info)
            client.policy({"checkpoint": checkpoint, "embodiment": body, "strict": True,
                           "action_hz": info["fps"], "sync_above_ms": 600})
            client.on_round_trip(adapted._capture_trip)
            (output / "right-model.json").write_text(json.dumps(
                {"checkpoint": checkpoint, "embodiment": body, "fps": info["fps"],
                 "state_names": adapted.state_names, "action_names": adapted.action_names,
                 "training_tasks": tasks, "gripper_scale": 100,
                 "smoothing": "dream.smoothing.SeamSmoother"}, indent=2) + "\n")
            return adapted
        except BaseException:
            client.close()
            raise

    def __init__(self, client, body, info):
        self.client, self.body, self.info = client, body, info
        self.state_names = info["features"]["observation.state"]["names"]
        self.action_names = info["features"]["action"]["names"]
        expected = {f"joint{i}.pos" for i in range(1, 7)} | {"gripper.pos"}
        for kind, names in (("state", self.state_names), ("action", self.action_names)):
            if len(names) != 7 or set(names) != expected:
                raise ValueError(f"Unsupported training {kind} columns")
            indices = [i for start, stop in body[kind].values() for i in range(start, stop)]
            if sorted(indices) != list(range(len(names))):
                raise ValueError(f"Model {kind} spans disagree with training metadata")
            if any(not name.startswith("right_") for name in body[kind]):
                raise ValueError("This adapter supports only the verified right-arm embodiment")
        if set(body["video"]) != {"overhead", "wrist_r"}:
            raise ValueError("Unexpected fine-tune camera schema")
        self._smoother = SeamSmoother(float(info["fps"]), pose_keys={"q_r_chunk": "state.right_arm"})
        self._trip_observation = None

    def __getattr__(self, name):
        return getattr(self.client, name)

    def _capture_trip(self, trip):
        self._trip_observation = dict(trip.observation)

    def _vector(self, parts, kind, names):
        vector = None
        for part, (start, stop) in self.body[kind].items():
            value = np.asarray(parts[f"{kind}.{part}"], dtype=float)
            if kind == "state":
                value = value.reshape(-1)
                if value.shape != (stop - start,):
                    raise ValueError("Invalid state part width")
                if vector is None:
                    vector = np.empty(len(names))
            else:
                # Scalar action parts may arrive as [H] rather than [H, 1].
                if value.ndim == 1 and stop - start == 1:
                    value = value[:, None]
                if value.ndim != 2 or value.shape[1] != stop - start:
                    raise ValueError(f"Invalid {part} action shape {value.shape}; expected rows of width {stop-start}")
                if vector is None:
                    vector = np.empty((len(value), len(names)))
                elif len(value) != len(vector):
                    raise ValueError("Action parts have different row counts")
            vector[..., start:stop] = value
        if vector is None or not np.isfinite(vector).all():
            raise ValueError("Missing or non-finite model values")
        return vector

    def set_observation(self, frames, observation, *, timestamp=None):
        # Metal EefWire already supplies right_gripper in the dataset's 0..100 units.
        joints = np.asarray(observation["state.right_arm"], dtype=float)
        grip = float(np.asarray(observation["state.right_gripper"]).reshape(-1)[0])
        values = {f"joint{i + 1}.pos": q for i, q in enumerate(joints)}
        values["gripper.pos"] = grip
        ordered = np.array([values[name] for name in self.state_names])
        mapped = {"prompt": observation["prompt"], **{
            f"state.{part}": ordered[start:stop].copy()
            for part, (start, stop) in self.body["state"].items()}}
        selected_frames = {f"video.{role}": frames[f"video.{role}"] for role in self.body["video"]}
        self.client.set_observation(selected_frames, mapped, timestamp=timestamp)

    def get_action(self, *, commanded=None):
        right_command = None if commanded is None else {
            key: value for key, value in commanded.items() if key == "q_r_chunk"}
        started = time.perf_counter()
        self._trip_observation = None
        try:
            reply = self.client.get_action(commanded=right_command)
        except dream.DreamSessionExpired:
            self._smoother.reset()
            raise
        if self._trip_observation is None:
            raise RuntimeError("Missing actual inference observation")
        vector = self._vector(reply["per_key"], "action", self.action_names)
        joints = vector[:, [self.action_names.index(f"joint{i}.pos") for i in range(1, 7)]]
        grip = np.clip(vector[:, self.action_names.index("gripper.pos")] / 100.0, 0.0, 1.0)
        q = np.column_stack([joints, grip])
        state = self._vector(self._trip_observation, "state", self.state_names)
        measured = state[[self.state_names.index(f"joint{i}.pos") for i in range(1, 7)]]
        # Apply the installed SDK's join after per_key actions become its q_r_chunk format.
        return self._smoother.smooth({"state.right_arm": measured}, {"q_r_chunk": q},
                                     started, time.perf_counter() - started, right_command)

    def reset_episode(self):
        self.client.reset_episode()
        self._smoother.reset()


def resample_rows(rows, source_hz, target_hz, horizon):
    """Match time coordinates, preserving speed; never extrapolate beyond a returned plan."""
    rows = np.asarray(rows, dtype=float)
    if rows.ndim != 2 or not np.isfinite(rows).all() or min(source_hz, target_hz) <= 0:
        raise ValueError("Invalid resampling input")
    source_t = np.arange(len(rows)) / source_hz
    target_t = np.arange(horizon) / target_hz
    if not len(rows) or target_t[-1] > source_t[-1] + 1e-9:
        raise ValueError("Target playback would exceed the returned plan's duration")
    return np.column_stack([np.interp(target_t, source_t, rows[:, j]) for j in range(rows.shape[1])])
