"""Call the Autoresearch lab's actor RPCs on a running stack without a TTY (`dimos shell` needs one).

    .venv/bin/python /work/PhyHillClimb/hfjobs/actor_rpc.py status
    .venv/bin/python /work/PhyHillClimb/hfjobs/actor_rpc.py start [model] [provider]
    .venv/bin/python /work/PhyHillClimb/hfjobs/actor_rpc.py stop

Run it from /work/dimos with the lab's venv so it joins the same bus the lab runs on.
"""

from __future__ import annotations

import sys

from dimos.porcelain.dimos import Dimos


def main() -> int:
    action = sys.argv[1] if len(sys.argv) > 1 else "status"
    app = Dimos.connect(timeout=20.0)
    try:
        labs = [info for info in app.list_modules() if any(r.name == "start_actor" for r in info.rpcs)]
        if not labs:
            print("no module with a start_actor RPC is running", file=sys.stderr)
            return 1
        info = labs[0]
        lab = getattr(app, info.instance_name or info.name)
        if action == "start":
            model = sys.argv[2] if len(sys.argv) > 2 else None
            provider = sys.argv[3] if len(sys.argv) > 3 else None
            print(lab.start_actor(model, provider))
        elif action == "stop":
            print(lab.stop_actor())
        else:
            print(lab.get_status())
        return 0
    finally:
        app.stop()


if __name__ == "__main__":
    raise SystemExit(main())
