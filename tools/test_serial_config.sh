#!/usr/bin/env bash
# Builds and runs the serial_config.h tests on this computer.
# Needs the ArduinoJson library PlatformIO installs (run `pio run` once first).
# Built without exceptions, like the firmware (ArduinoJson 5 relies on it).
set -euo pipefail
cd "$(dirname "$0")/.."

ARDUINOJSON=${ARDUINOJSON:-.pio/libdeps/linka/ArduinoJson}
if [ ! -f "$ARDUINOJSON/ArduinoJson.h" ]; then
  echo "ArduinoJson not found in $ARDUINOJSON: run 'pio run' first, or set ARDUINOJSON" >&2
  exit 1
fi

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
g++ -std=c++17 -fno-exceptions -Wall -Wextra -Wno-unused-function -I"$ARDUINOJSON" \
  tools/test_serial_config.cpp -o "$out/test_serial_config"
"$out/test_serial_config"
