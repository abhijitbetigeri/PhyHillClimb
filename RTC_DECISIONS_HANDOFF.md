# Fine-tuned model: RTC and Decisions API handoff

## RTC — reuse this implementation

Robot PC code directory:
`/home/pah/hackathon_data/evaluations/base-unplug-20261108/`

The source is now included under [`tools/`](tools/RTC_DECISIONS.md). The repository
version adds right-only fine-tuned stacking with the Decisions gate; the earlier
station experiment directory may still contain the older base-only stack preset.

- **Requested policy is fine-tuned v1.** For right-arm unplug + stacking, use the multitask checkpoint **`dream-1-ft:2d43713b-right-arm-v1`**. The separate **`dream-1-ft:2d43713b-right-unplug-v1`** was trained for unplug only; do not use it as the stacking checkpoint. Do not silently fall back to the base model.
- **Reuse `fine_tuned_client.py` (`RightFineTuneClient`) with `sdk_bimanual.py` and the installed `robots.control.ControlLoop`.** The old custom playback loop skipped action rows based on wall-clock/API latency and caused abrupt gripper changes. Keep the corrected SDK executor.
- Keep SDK `client.refill_due` pacing, the executed/commanded action anchor, and SDK chunk installation. Do not skip an entire latency interval when the arms were holding; skip only rows actually executed during inference.
- Read the actual dataset fps and model embodiment before loading the checkpoint. Preserve the settings below, existing soft limits and target-step limits; no gripper force increase was made.
- The fine-tuned API returns `per_key` joint/gripper actions. `RightFineTuneClient` maps them into `q_r_chunk` and applies the installed **`dream.smoothing.SeamSmoother`**, using the actual inference observation and commanded anchor. Reset both the Dream session and smoother at task changes. Gripper output can have shape `[H]` or `[H,1]`.
- Convert fine-tuned gripper output to SDK units exactly once (scales below). The Metal observation wire already supplies gripper state in dataset units; do NOT multiply it again. Dataset column order comes from the matching `meta/info.json`; policy cameras are `overhead` and `wrist_r`.
- If the left arm still uses a base policy, reuse `dual_client.py`: resample its trajectory to the SAME physical timestamps at the fine-tuned playback rate. Do not accelerate it by directly playing lower-rate rows faster.
- The verified local metadata archive is `/home/pah/hackathon_data/evaluations/right-unplug-v1-20261108/training.zip` (unplug-only). For multitask v1, obtain/verify its matching training metadata and original task prompts from `dlqkstjr10/metal-right-arm-multitask-20261107`; do not assume the unplug-only archive is the multitask training export.
- At task changes, discard the old action queue and reset the policy episode safely after any in-flight request finishes. Never execute an old unplug response during stacking.

| RTC setting | Value |
|---|---|
| Fine-tuned native/playback rate; horizon | 30 Hz; 48 rows |
| Fine-tuned synchronous pacing threshold | 600 ms median RTT |
| Command writer; maximum interpolation window | 250 Hz; 0.1 s |
| Maximum gripper change per playback tick | 0.03333 at 30 Hz |
| Maximum end-effector speed | 0.08 m/s |
| Startup ramp; ramp speed cap | 2 s; 0.2 rad/s |
| Existing gripper force cap | 0.65 (SDK units) |
| Fine-tuned gripper state/output; SDK action scale | 0–100; 0–1 |
| Optional base left-arm native rate | 20 Hz, resampled to 30 Hz without speedup |

## Decisions API — stop execution, then hold

- The repository's **`tools/sdk_bimanual.py --stack-three`** defaults to the right-arm multitask fine-tune and requires its matching `--training-archive`. It combines `RightFineTuneClient`, a left-arm hold, and `stack_completion.py`; no base fallback is allowed. `--right-only --right-checkpoint` also supports fine-tuned execution without a stack gate. The independent two-arm cable mode remains separate.
- The existing visual gate detects a three-block tower only. Unplug completion criteria and automatic unplug-to-stack transition still need to be connected by the next operator.
- Endpoint/model: **`POST https://api.openai.com/v1/decisions`**, **`gpt-6-luna`**. Load the existing key from **`/home/pah/.dreamlabs/openai.env`**; do not copy the key into code or this document.
- Use all camera views and separate face-blurred monitor images. The first fresh result meeting both thresholds below triggers a policy stop. A partial tower or a held top block is incomplete.
- Stop order: **cancel the SDK runner → stop and join the command writer → clear buffered actions → hold the measured paired pose → drain/discard late inference**. Do not wait for an API reply before holding. See `stop_policy()`.
- Confirm again while held using a fresh frame. Confirmation never runs additional policy actions. Uncertain confirmation stays held; no automatic resume. Stale/missing camera data, API failure, or an exceeded decision deadline also stops policy execution. The held interval now matches the repository's competition requirement; this is a camera-based check, not a physical stability guarantee.
- Completion holds position with motors powered; it does **not** automatically park. An explicit operator STOP requests verified paired parking. Never disable these brakeless arms directly on completion or fault.

| Decisions setting / evidence | Value |
|---|---|
| Required tower; monitor views | 3 released blocks; overhead + both wrists |
| Monitor image short side | 480 pixels |
| Placement / grippers-clear thresholds | >= 0.90 / >= 0.95 |
| Repository held confirmation interval | 3 s |
| Competition stability requirement in AGENTS.md | 3 s after release |
| Decision age limit / watchdog deadline | 2 s |
| Hardware-free regression tests passed | 33 |
| Saved-image negative control | 2-block scene: 0.06 completion probability; 1.26 s RTT |

Tests cover fine-tuned unit conversion, ordering and time-preserving resampling. A mixed right-unplug-v1/base dry run succeeded without motor commands. Fine-tuned multitask + Decisions end-to-end physical completion is not yet validated. Camera/API latency is not instantaneous.
