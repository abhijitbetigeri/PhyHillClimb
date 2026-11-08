WORKING SET: real-arm unplug, 8/8 plugs out (Sun 8 Nov 2026, 12:55-13:40 PST)
Model: dream-1-ft:2d43713b-right-arm-v1-plus500 (Dream Labs, hosted). Runner: dream_right_arm_run.py in this folder.
Run (from ~/Superfast-harness, hand on the e-stop, strip unplugged from the wall, cables seated, nothing loose on the strip):
  ~/dream-env/bin/python ~/PhyHillClimb-tools/working_set_unplug_8of8/dream_right_arm_run.py --checkpoint dream-1-ft:2d43713b-right-arm-v1-plus500 --live --seconds 150 --pull-hold-torque 1.5
Keep the model warm (one call every 4 min): nohup ~/PhyHillClimb-tools/keep_warm.sh right-arm-v1-plus500 120 &
Behaviour: model reach -> settle 2 s -> close at the model pose -> demo-1 pull with hold torque 0.75->1.5 N.m ->
sustained lean-in until the shoulder catches up -> pan -0.4 rad -> open and drop. Misses reopen and retry.
Files are read-only; copy before editing. The live copy ~/PhyHillClimb-tools/dream_right_arm_run.py may move on.
