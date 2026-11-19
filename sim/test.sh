#!/bin/sh
# Runs the committed ZeroPilot 4.0 plane image (fw/stm32h753iitx.elf) on the simulated STM32H753 with all
# Python peers for 20 virtual s. Pass = the ground-station peer decoded every MAVLink message the firmware
# sends on USART1 with valid CRCs, the heartbeat reports armed, and logs/system0.log exists on the SD card.
# Needs python3 (+pyyaml) and mtools (mformat, mmd, mdir).
cd "$(dirname "$0")/.." || exit 1
SIM=${SIM:-sim}
cd sim || exit 1
O=out
rm -rf "$O"
trap 'rm -rf "$O"' EXIT
./make_card.sh "$O/card.img" || exit 1
"$SIM" run --scenario scenario.yaml --timeout 20 --output "$O/out.txt" --frames "$O/frames.txt" 2> "$O/log.txt"
rc=$?
fail=0
chk() { if eval "$2"; then echo "ok   $1"; else echo "FAIL $1"; fail=1; fi; }
chk "sim exited 0" "[ $rc = 0 ]"
for m in HEARTBEAT GPS_RAW_INT RAW_IMU SCALED_PRESSURE ATTITUDE SERVO_OUTPUT_RAW RC_CHANNELS DISTANCE_SENSOR BATTERY_STATUS ARMED; do
  chk "MAVLINK $m decoded" "grep -qF '[gcs] MAVLINK $m' '$O/log.txt'"
done
chk "no bad MAVLink CRC" "! grep -qF 'MAVLINK bad crc' '$O/log.txt'"
chk "logs/system0.log on card" "mdir -i '$O/card.img' ::logs | grep -q system0"
chk "no fault" "! grep -qiE 'hardfault|panic|guru' '$O/log.txt'"
exit $fail
