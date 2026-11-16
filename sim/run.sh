#!/usr/bin/env bash
# Run the ZeroPilot plane image on the simulated STM32H753 with every peer attached,
# then check the MAVLink it sent and the log file it wrote to the SD card.
#
#   ./run.sh [virtual-seconds]            # default 20; kinematic plant
#   JSBSIM_SITE=<venv>/lib/python3.x/site-packages ./run.sh 30   # flown by JSBSim (pip install jsbsim)
#
# Outputs in sim/out/: out.txt (UART bytes, hex), frames.txt (SPI/I2C/CAN), log.txt (peer log),
# view.txt (decoded MAVLink), card.img (the SD card after the run).
set -euo pipefail
cd "$(dirname "$0")"
T=${1:-20}
ELF=../stm32h753iitx/build-h753/stm32h753iitx.elf
[ -f "$ELF" ] || { echo "$ELF missing: run ./build.sh first" >&2; exit 1; }
[ -z "${JSBSIM_SITE:-}" ] || export PYTHONPATH="$JSBSIM_SITE"
./make_card.sh out/card.img
sim run --scenario scenario.yaml --timeout "$T" --output out/out.txt --frames out/frames.txt 2> out/log.txt
python3 view.py out/out.txt | tee out/view.txt

fail=0
for m in HEARTBEAT GPS_RAW_INT RAW_IMU SCALED_PRESSURE ATTITUDE SERVO_OUTPUT_RAW RC_CHANNELS \
         DISTANCE_SENSOR BATTERY_STATUS ARMED; do
  grep -q "\[gcs\] MAVLINK $m" out/log.txt || { echo "MISSING MAVLINK $m"; fail=1; }
done
! grep -q "MAVLINK bad crc" out/log.txt || { echo "BAD MAVLink CRC"; fail=1; }
mdir -i out/card.img ::logs | grep -q 'system0' || { echo "MISSING logs/system0.log on the card"; fail=1; }
echo "card:"; mdir -i out/card.img ::logs | grep system
[ $fail = 0 ] && echo "PASS" || { echo "FAIL"; exit 1; }
