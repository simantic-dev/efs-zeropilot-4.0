#!/usr/bin/env bash
# Build the plane image for the STM32H753IITX from a command line (needs the Arm GNU toolchain, cmake, ninja).
# Drops stm32h753iitx/build-h753/stm32h753iitx.elf.
set -euo pipefail
ZP=$(cd "$(dirname "$0")/.." && pwd)
( cd "$ZP" && git submodule update --init --recursive --depth 1 )
( cd "$ZP/zeropilot4.0" && ./hwbuild.bash -v plane -b h753iit )   # libzeropilot4.0.a
cmake -G Ninja -S "$ZP/stm32h753iitx" -B "$ZP/stm32h753iitx/build-h753" -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE="$ZP/stm32h753iitx/stm32h753iitx.cmake" -DVEHICLE=PLANE
cmake --build "$ZP/stm32h753iitx/build-h753"
