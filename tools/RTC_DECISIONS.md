# Fine-tuned RTC execution and Decisions stop gate

These are the station's SDK executor, fine-tuned action adapter, optional independent
arm clients, completion monitor, and hardware-free regression tests. Run on NUCK-4
using its existing environment; the files contain no credentials.

## Runtime

| Dependency | Station value |
|---|---|
| Python environment | `/home/pah/dream-env/bin/python` |
| SDK packages checked | `dreamlabs==1.0.37`, `dreamlabs-client==1.1.3` |
| Paired hardware/parking and monitor helpers | `/home/pah/Superfast-harness` (`embodiments.metal.teleop`, `supervised`, `completion`) |
| Kit and camera configuration | `/home/pah/physical-ai-hacks/configs/nuck4-subtasks.yaml` |
| CAN environment | `/home/pah/Superfast-harness/.env` |
| Dream credentials / Decisions key | `~/.dreamlabs/credentials` / `~/.dreamlabs/openai.env` |

The installed harness is a required station dependency, not vendored here. Its
`CompletionMonitor` supplies the face-blurred snapshot and Decisions HTTP backend;
`StackMonitor` supplies the stop/confirmation rules. Preserve the paired park
adapter: the Metal arms are brakeless. The default mode does not enable motors,
but it opens camera devices and read-only CAN connections, so the station must be
available even for a dry run.

## Fine-tuned use

From this repository root on the robot PC, test the unplug-only adapter without
motor commands:

```bash
~/dream-env/bin/python tools/sdk_bimanual.py \
  --right-only --right-checkpoint dream-1-ft:2d43713b-right-unplug-v1 \
  --seconds 8 --max-api-calls 2 --output /tmp/rtc-unplug-dry
```

The stack preset selects `dream-1-ft:2d43713b-right-arm-v1` and holds the left arm.
Supply the matching multitask export ZIP; no unrelated archive is substituted:

```bash
~/dream-env/bin/python tools/sdk_bimanual.py \
  --stack-three --right-checkpoint dream-1-ft:2d43713b-right-arm-v1 \
  --training-archive /absolute/path/to/matching-training.zip \
  --seconds 120 --output /tmp/rtc-stack-dry
```

Every output directory must be new. `--task` can supply the original training task
prompt. Only the operator adds `--live` for physical execution. Failed model or
metadata loading never falls back to the base model. `right-model.json` records
the actual checkpoint, embodiment, tensor columns and training tasks.

`--separate-arms --right-checkpoint ...` retains the earlier two-stream cable
mode: a fine-tuned right policy and a base left policy. It cannot be combined
with `--stack-three`. Unplug completion and automatic transition to stacking are
still the relay director's integration work; these scripts do not implement
that sequence.

## Stop behavior

The first fresh tower-and-release candidate cancels policy execution, joins the
command writer, clears queued actions and holds the measured paired pose before
draining any in-flight inference. A fresh held-state check confirms completion;
uncertainty never resumes the policy. The decision watchdog also stops on an
overdue request. Completion and the time limit keep motors powered and the arm
lease held until an explicit STOP/park on the existing dashboard.

| Setting | Value |
|---|---|
| Fine-tuned playback / horizon | 30 Hz / 48 rows |
| Fine-tuned synchronous pacing threshold | 600 ms median RTT |
| Gripper step limit | 0.03333 per playback tick |
| Command writer / interpolation cap | 250 Hz / 0.1 s |
| Decisions model / endpoint | `gpt-6-luna` / `POST /v1/decisions` |
| Placement / release thresholds | 0.90 / 0.95 |
| Held confirmation wait | 3 s |
| Maximum decision age / request watchdog | 2 s |
| Operator dashboard | `http://127.0.0.1:8091` on the robot PC |
| Hardware-free tests passed on the installed station SDK | 33 |

## Hardware-free checks

```bash
cd tools
PYTHONPATH=/home/pah/Superfast-harness ~/dream-env/bin/python -m unittest \
  test_sdk_bimanual test_stack_completion test_dual_client test_fine_tuned_client
```

These checks use mocks and the installed SDK; they open no cameras, CAN devices
or model sessions. They verify playback pacing, unit conversion, independent
streams, late-reply discard, stop ordering and fine-tuned selection without a
base fallback. End-to-end physical multitask completion remains unverified.
