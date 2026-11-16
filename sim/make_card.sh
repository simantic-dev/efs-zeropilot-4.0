#!/usr/bin/env bash
# Blank 16 MiB FAT16 card image (needs mtools). Not committed.
set -euo pipefail
OUT=${1:?usage: make_card.sh <card.img>}
mkdir -p "$(dirname "$OUT")"; rm -f "$OUT"
mformat -C -T 32768 -h 16 -s 32 -v ZP -i "$OUT" ::
# Pre-create logs/. On a blank card the firmware's own f_mkdir("logs") runs (6.10 s) but its block writes never
# reached the image file on this sim build (see README, gaps).
mmd -i "$OUT" ::logs
