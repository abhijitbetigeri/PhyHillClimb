# Run the fine-tuned Dream-1 unplug model on the right Metal arm (NUCK-4)

Everything is on this machine. The model runs on Dream Labs servers; this runner sends camera frames and joints and
plays the replies on the right arm. Left arm is held still.

## Before every run
1. Nothing else holds the arms:  pgrep -fa "leader_collect|dhk-collect|dreamcli|teleop"   (must print nothing)
   If something does, ask its owner to stop it (Ctrl-C). If you then get "gripper motor is not disabled",
   support the arm and power-cycle both arms.
2. Bench: strip centred on the table, its own power cord OUT of the wall, the red/orange cables pushed fully into
   the strip, cubes/sticks/cups well away from the strip, nobody leaning into the camera views.
3. Right arm parked upright at rest. A person with a hand on the e-stop.

## Run
    cd ~/Superfast-harness
    ~/dream-env/bin/python ~/PhyHillClimb-tools/dream_right_arm_run.py --checkpoint dream-1-ft:2d43713b-right-arm-v1-plus500 --live --seconds 120

Expect "cameras delivering frames", then "arms synced at rest; starting the model", then "chunk: 48 rows ..." lines.
Dry run (nothing moves): replace --live with --dry-run.
Other model: dream-1-ft:2d43713b-right-unplug-v1 (unplug-only).

## If something goes wrong
- "pool_warming": the model is loading (up to 50 min). Ctrl-C and wait; keep_warm.sh keeps both models loaded:
    nohup ~/PhyHillClimb-tools/keep_warm.sh right-arm-v1-plus500 120 >/dev/null 2>&1 &
- "unsafe instruction in this scene": Dream safety filter; unplug the strip from the wall, remove loose cables.
- "startup sync stalled on gripper": tell Abhijit (the grip fix did not take).
- Press the e-stop if the jaw presses hard into the strip/table or heads toward the cubes.

Logs of every run: ~/PhyHillClimb-runs/right_arm_<time>_<live|dry>.jsonl

## LiveKit relay (dashboard for the team and the judges)
Add `--relay` to any run command above. The runner then writes its camera frames, arm state and status to
/dev/shm/phyhill-relay and the bridge (tmux session `relay`, venv ~/relay-env, folder ~/PhyHillClimb-relay) publishes
them to LiveKit Cloud room `relay-nuck4` as the Portal robot; nothing about the control loop changes and the bridge
never commands the arm. Dashboard: http://10.103.1.45:8090/viewer.html (links in Abhijit's notes/relay-links.md).
Bridge not running?  tmux new -d -s relay -c ~/PhyHillClimb-relay "~/relay-env/bin/python -m relay.station_bridge --room relay-nuck4 2>&1 | tee -a bridge.log"
