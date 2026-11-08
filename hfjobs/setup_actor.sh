#!/usr/bin/env bash
# Inside the HF job: install what Dimensional's Autoresearch actor (`start_actor` RPC) needs: Node 24 and the Pi
# coding agent (`@earendil-works/pi-coding-agent`, the version dimos/agents/package.json pins), both reachable from
# the lab's PATH ($HOME/.local/bin, which restart_lab.sh puts first). The provider key is NOT handled here: write it
# into /work/dimos/.env separately (dimos run loads it), never through a logged command line.
#   bash /work/PhyHillClimb/hfjobs/setup_actor.sh
set -euo pipefail
NODE_VERSION=${NODE_VERSION:-24.11.1}
PI_VERSION=${PI_VERSION:-$(python3 -c "import json;print(json.load(open('/work/dimos/dimos/agents/package.json'))['dependencies']['@earendil-works/pi-coding-agent'])")}
mkdir -p "$HOME/.local/bin" /opt
if [ ! -x /opt/node/bin/node ]; then
  echo "installing node $NODE_VERSION"
  curl -fsSL "https://nodejs.org/dist/v$NODE_VERSION/node-v$NODE_VERSION-linux-x64.tar.xz" -o /tmp/node.tar.xz
  mkdir -p /opt/node && tar xJf /tmp/node.tar.xz -C /opt/node --strip-components=1 && rm /tmp/node.tar.xz
fi
ln -sf /opt/node/bin/node /opt/node/bin/npm /opt/node/bin/npx "$HOME/.local/bin/"
export PATH="/opt/node/bin:$HOME/.local/bin:$PATH"
echo "node $(node --version), npm $(npm --version)"
if ! command -v pi >/dev/null || [ "$(pi --version 2>/dev/null | grep -o '[0-9][0-9.]*' | head -1)" != "$PI_VERSION" ]; then
  echo "installing @earendil-works/pi-coding-agent@$PI_VERSION"
  npm install -g --silent "@earendil-works/pi-coding-agent@$PI_VERSION"
fi
ln -sf /opt/node/bin/pi "$HOME/.local/bin/pi"
echo "pi $(pi --version 2>&1 | head -1) at $(command -v pi)"
