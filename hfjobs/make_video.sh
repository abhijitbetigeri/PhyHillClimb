#!/usr/bin/env bash
# Turn a session's scene-camera frames into an MP4 (inside the job or anywhere with ffmpeg).
#   bash make_video.sh <frames dir> <out.mp4> [fps]
set -euo pipefail
DIR=${1:?frames dir}; OUT=${2:?out.mp4}; FPS=${3:-10}
command -v ffmpeg >/dev/null || { apt-get install -y -qq ffmpeg >/dev/null 2>&1 || true; }
ls "$DIR"/*.jpg | sort > /tmp/frames.txt
ffmpeg -loglevel error -y -r "$FPS" -f concat -safe 0 -i <(sed 's/^/file /' /tmp/frames.txt) \
  -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -pix_fmt yuv420p -crf 23 "$OUT"
echo "$OUT $(wc -l < /tmp/frames.txt) frames"
