"""Local, publish-only LiveKit presentation demo using recorded simulation clips.

    .venv/bin/python -m relay.demo --open

Owns only its own local server and bridge; Ctrl-C cleans both up. No station,
Dream credentials, paid service, or motor driver is opened.
"""
from __future__ import annotations

import argparse
import functools
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlencode
import uuid
import webbrowser

import cv2
import numpy as np
from PIL import Image, ImageDraw

from relay.common import ROOT, LOCAL_KEY, LOCAL_SECRET
from tools.relay_tap import RelayTap, portal_state


def card(title: str, subtitle: str) -> np.ndarray:
    image = Image.new('RGB', (640, 360), '#101923')
    draw = ImageDraw.Draw(image)
    draw.text((24, 120), title, fill='#e8ecf1', font_size=26)
    draw.text((24, 170), subtitle, fill='#aab7c6', font_size=18)
    return np.asarray(image).copy()


class Replay:
    def __init__(self, path: Path):
        self.cap = cv2.VideoCapture(str(path))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not self.cap.isOpened() or self.fps <= 0 or self.count <= 0:
            self.cap.release()
            raise ValueError(f'Cannot decode demo clip: {path}')
        self.duration = self.count / self.fps
        self.index = -1
        self.frame = None

    def read(self, elapsed: float) -> np.ndarray:
        index = min(self.count - 1, max(0, int(elapsed * self.fps)))
        if index != self.index:
            if index != self.index + 1:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, bgr = self.cap.read()
            if not ok:
                raise RuntimeError(f'Video decoding failed at frame {index}')
            self.frame = cv2.cvtColor(cv2.resize(bgr, (640, 360)), cv2.COLOR_BGR2RGB)
            self.index = index
        return self.frame

    def close(self):
        self.cap.release()


def available(port: int):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', port))


def wait_server(port: int, process: subprocess.Popen):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('LiveKit server exited; see server.log')
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=.2):
                return
        except OSError:
            time.sleep(.1)
    raise TimeoutError('Local LiveKit server did not become ready')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--open', action='store_true', help='open the local dashboard')
    ap.add_argument('--seconds', type=float, default=0, help='0 loops until Ctrl-C')
    ap.add_argument('--port', type=int, default=17880)
    ap.add_argument('--web-port', type=int, default=18090)
    ap.add_argument('--cable-video', type=Path, default=ROOT/'runs/videos/hq_cable_v1_short.mp4')
    ap.add_argument('--tower-video', type=Path, default=ROOT/'runs/videos/hq_tower_v24_short.mp4')
    args = ap.parse_args()
    binary = shutil.which('livekit-server')
    if not binary:
        ap.error('livekit-server is required (brew install livekit)')
    if args.seconds < 0:
        ap.error('--seconds must be nonnegative')
    for port in (args.port, args.port+1, args.web_port):
        available(port)
    clips = []
    try:
        for path in (args.cable_video, args.tower_video):
            clips.append(Replay(path))
    except Exception:
        for clip in clips:
            clip.close()
        raise
    room = 'relay-demo-' + uuid.uuid4().hex[:8]
    directory = ROOT/'runs/livekit_demo'/room
    directory.mkdir(parents=True)
    env = dict(os.environ, LIVEKIT_URL=f'ws://127.0.0.1:{args.port}', LIVEKIT_API_KEY=LOCAL_KEY,
               LIVEKIT_API_SECRET=LOCAL_SECRET, LIVEKIT_ROOM=room, LIVEKIT_ROBOT_TOKEN='',
               RELAY_TAP_DIR=str(directory/'tap'))
    # Override inherited cloud settings before importing the bridge or minting tokens.
    os.environ.update({k:v for k,v in env.items() if k.startswith('LIVEKIT_')})
    from livekit import api

    class Handler(SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/demo':
                token = (api.AccessToken(LOCAL_KEY, LOCAL_SECRET)
                         .with_identity('viewer-' + uuid.uuid4().hex[:8])
                         .with_grants(api.VideoGrants(room_join=True, room=room,
                                                    can_publish=False, can_subscribe=True)).to_jwt())
                self.send_response(302)
                self.send_header('Location', '/viewer.html#' + urlencode({'url':env['LIVEKIT_URL'], 'token':token}))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
            else:
                super().do_GET()

        def log_message(self, *args):
            pass

    http = None
    children, handles = [], []
    tap = None
    try:
        for name in ('server', 'bridge'):
            handles.append((directory/f'{name}.log').open('w'))
        server = subprocess.Popen([binary, '--dev', '--bind', '127.0.0.1',
                                   '--port', str(args.port), '--rtc.tcp_port', str(args.port+1),
                                   '--udp-port', str(args.port+2)], env=env,
                                  stdout=handles[0], stderr=subprocess.STDOUT)
        children.append(server)
        wait_server(args.port, server)
        tap = RelayTap(directory/'tap')
        bridge = subprocess.Popen([sys.executable, '-m', 'relay.station_bridge', '--room', room,
                                   '--tap-dir', str(directory/'tap')], cwd=ROOT, env=env,
                                  stdout=handles[1], stderr=subprocess.STDOUT)
        children.append(bridge)
        http = ThreadingHTTPServer(('127.0.0.1', args.web_port), functools.partial(Handler, directory=str(ROOT/'web')))
        threading.Thread(target=http.serve_forever, daemon=True).start()
        url = f'http://127.0.0.1:{args.web_port}/demo'
        (directory/'demo.json').write_text(json.dumps({'url':url,'room':room,'livekit_url':env['LIVEKIT_URL'],
                                                      'source':'recorded-simulation','status':'scripted'}, indent=2))
        print(f'Dashboard: {url}\nRoom: {room}\nLogs: {directory}\nRecorded simulation; scripted status; publish-only. Ctrl-C stops the demo.', flush=True)
        if args.open:
            webbrowser.open(url)
        idle = card('Wrist camera unavailable', 'This demo replays one recorded simulation view.')
        start = time.monotonic()
        cable_s, tower_s = (c.duration for c in clips)
        cycle_s = cable_s + tower_s + 8
        with (directory/'events.jsonl').open('w') as log:
            previous = None
            while not args.seconds or time.monotonic()-start < args.seconds:
                if any(p.poll() is not None for p in children):
                    raise RuntimeError('Demo process exited; see server.log / bridge.log')
                elapsed = (time.monotonic()-start) % cycle_s
                stage = 'unplug' if elapsed < cable_s else 'stack' if elapsed < cable_s+tower_s else 'done'
                frame = clips[0].read(elapsed) if stage=='unplug' else clips[1].read(min(tower_s-.01, elapsed-cable_s))
                frame = frame.copy()
                cv2.rectangle(frame, (0, 0), (640, 34), (16, 25, 35), -1)
                cv2.putText(frame, 'RECORDED SIMULATION | SCRIPTED DEMO', (12, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 200, 80), 1)
                status = {'mode':'demo-replay','phase':stage,'stage':stage,'task':'recorded relay demo',
                          'elapsed_s':round(min(elapsed,cable_s+tower_s),1),'unplugged':int(stage!='unplug'),
                          'placed':2 if stage=='done' else 0,
                          'event':'Recorded simulation; scripted progression, no live policy or motor control'}
                tap.publish({'overhead':frame,'wrist_l':idle,'wrist_r':idle}, portal_state([0.]*6, 1.), status)
                if stage != previous:
                    log.write(json.dumps({'event':'stage','stage':stage,'elapsed_s':elapsed,'scripted':True})+'\n')
                    log.flush()
                    previous = stage
                time.sleep(1/15)
    except KeyboardInterrupt:
        pass
    finally:
        if http:
            http.shutdown()
            http.server_close()
        if tap:
            tap.close()
        for process in reversed(children):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for handle in handles:
            handle.close()
        for clip in clips:
            clip.close()


if __name__ == '__main__':
    main()
