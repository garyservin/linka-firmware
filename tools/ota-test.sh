#!/usr/bin/env bash
# Local OTA server for testing firmware on a real sensor, without touching production.
#
# Runs linka-ota-server on this machine over HTTPS (self-signed, the firmware doesn't
# verify certificates) with its own firmware directory, plus a fake measurements API on
# /api that checks and prints each report (tools/ota_test_app.py). Point the sensor's
# "Server for OTA upgrades" and "URL for the backend" (to keep test reports out of
# AireLibre) at the URLs printed by `serve`.
#
# Usage: tools/ota-test.sh <command> [args]
#   setup [release...]        Create the test directory with a certificate and the given
#                             linka-firmware releases (default: the latest release)
#   serve                     Run the local OTA server (foreground, Ctrl-C to stop)
#   add [version]             Build this checkout as <version> and make it `latest`
#                             (default version: git describe + timestamp)
#   add-release <version>     Download a published release and add it (not made latest)
#   latest <version>          Make <version> the one served to sensors (e.g. to roll back)
#   list                      Show the available versions and `latest`
#   check [version]           Ask the server for an update like a sensor running <version>
#   flash <version> [--erase] Flash <version> over USB (--erase wipes WiFi and settings too)
#
# Environment:
#   LINKA_TEST_DIR    test directory (default ~/linka-test)
#   LINKA_OTA_SERVER  linka-ota-server checkout (default ../linka-ota-server)
#   PORT              HTTPS port (default 5443)
#   UPLOAD_PORT       serial port for `flash` (default: auto-detected by esptool)
set -euo pipefail

FIRMWARE_DIR=$(cd "$(dirname "$0")/.." && pwd)
TEST_DIR=${LINKA_TEST_DIR:-$HOME/linka-test}
BIN_DIR=$TEST_DIR/bin
OTA_SERVER=${LINKA_OTA_SERVER:-$FIRMWARE_DIR/../linka-ota-server}
PORT=${PORT:-5443}
REPO=garyservin/linka-firmware

die() { echo "error: $*" >&2; exit 1; }
bin_name() { echo "linka-firmware-v$1.bin"; }
need_setup() { [ -f "$BIN_DIR/versions" ] || die "run '$0 setup' first"; }

lan_ip() {
  ip route get 1.1.1.1 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") print $(i + 1)}'
}

# versions_update <version> <file> <md5> <set_latest:true|false>
versions_update() {
  python3 - "$BIN_DIR/versions" "$@" <<'EOF'
import json, os, sys
path, version, file, md5, latest = sys.argv[1:]
versions = json.load(open(path)) if os.path.exists(path) else {}
if file:
    versions[version] = {"file": file, "md5sum": md5}
if version not in versions:
    sys.exit(f"error: version {version} not found, see 'list'")
if latest == "true":
    versions["latest"] = version
with open(path + ".tmp", "w") as f:
    json.dump(versions, f, indent=4, sort_keys=True)
    f.write("\n")
os.replace(path + ".tmp", path)
EOF
}

add_release() {
  local version=${1#v} set_latest=${2:-false} bin
  bin=$(bin_name "$version")
  if ! gh release download "$version" -R "$REPO" -p "$bin" -D "$BIN_DIR" --clobber 2>/dev/null &&
     ! gh release download "v$version" -R "$REPO" -p "$bin" -D "$BIN_DIR" --clobber; then
    die "release $version has no $bin"
  fi
  versions_update "$version" "$bin" "$(md5sum < "$BIN_DIR/$bin" | cut -d' ' -f1)" "$set_latest"
  echo "Added release $version"
}

cmd_setup() {
  mkdir -p "$BIN_DIR"
  if [ ! -f "$TEST_DIR/cert.pem" ]; then
    openssl req -x509 -newkey rsa:2048 -nodes -subj /CN=linka-test -days 365 \
      -keyout "$TEST_DIR/key.pem" -out "$TEST_DIR/cert.pem" 2>/dev/null
    echo "Created self-signed certificate in $TEST_DIR"
  fi
  local releases=("$@")
  [ ${#releases[@]} -gt 0 ] || releases=("$(gh release view -R "$REPO" --json tagName -q .tagName)")
  for release in "${releases[@]}"; do
    add_release "$release"
  done
  # Serve the first one until a test build is added
  versions_update "${releases[0]#v}" "" "" true
  cmd_list
}

cmd_serve() {
  need_setup
  [ -d "$OTA_SERVER/linka_ota_server" ] || die "linka-ota-server not found in $OTA_SERVER, set LINKA_OTA_SERVER"
  local ip
  ip=$(lan_ip)
  cat <<EOF
Set these in the sensor's setup page:
  Server for OTA upgrades: https://${ip:-<this-machine-ip>}:$PORT/ota
  URL for the backend:     https://${ip:-<this-machine-ip>}:$PORT/api   (reports are checked and printed here)

EOF
  cmd_list
  echo
  cd "$OTA_SERVER"
  FIRMWARE_PATH=$BIN_DIR APP_VERSION=local-test exec uv run gunicorn \
    --certfile "$TEST_DIR/cert.pem" --keyfile "$TEST_DIR/key.pem" \
    --bind "0.0.0.0:$PORT" --access-logfile - --pythonpath "$FIRMWARE_DIR/tools" 'ota_test_app:app'
}

cmd_add() {
  need_setup
  local version=${1:-} bin
  if [ -z "$version" ]; then
    version="$(git -C "$FIRMWARE_DIR" describe --tags --dirty --match 'v[0-9]*' --match '[0-9]*' 2>/dev/null \
               | sed 's/^v//')-t$(date +%H%M%S)"
  fi
  version=${version#v}
  bin=$(bin_name "$version")
  (cd "$FIRMWARE_DIR" && LINKA_VERSION=$version pio run -s)
  cp "$FIRMWARE_DIR/.pio/build/linka/firmware.bin" "$BIN_DIR/$bin"
  versions_update "$version" "$bin" "$(md5sum < "$BIN_DIR/$bin" | cut -d' ' -f1)" true
  echo "Built $version, now served as latest. Reboot the sensor to update it."
}

cmd_add_release() {
  need_setup
  [ $# -eq 1 ] || die "usage: $0 add-release <version>"
  add_release "$1"
}

cmd_latest() {
  need_setup
  [ $# -eq 1 ] || die "usage: $0 latest <version>"
  versions_update "${1#v}" "" "" true
  echo "Now serving ${1#v} as latest. Reboot the sensor to update it."
}

cmd_list() {
  need_setup
  python3 - "$BIN_DIR/versions" <<'EOF'
import json, sys
versions = json.load(open(sys.argv[1]))
latest = versions.pop("latest", None)
for version, info in versions.items():
    print(f"{'*' if version == latest else ' '} {version:32} {info['file']}  {info['md5sum']}")
print("(* = latest, served to sensors)")
EOF
}

cmd_check() {
  need_setup
  local version=${1:-0.0.0} md5=none code
  if [ -n "${1:-}" ] && [ -f "$BIN_DIR/$(bin_name "${1#v}")" ]; then
    md5=$(md5sum < "$BIN_DIR/$(bin_name "${1#v}")" | cut -d' ' -f1)
  fi
  code=$(curl -ksS -o /dev/null -w '%{http_code}' "https://127.0.0.1:$PORT/ota" \
    -H 'User-Agent: ESP8266-http-Update' \
    -H 'x-ESP8266-STA-MAC: 18:FE:AA:AA:AA:AA' -H 'x-ESP8266-AP-MAC: 1A:FE:AA:AA:AA:AA' \
    -H 'x-ESP8266-free-space: 671744' -H 'x-ESP8266-sketch-size: 513472' \
    -H "x-ESP8266-sketch-md5: $md5" -H 'x-ESP8266-chip-size: 4194304' \
    -H 'x-ESP8266-sdk-version: 2.2.2' -H "x-ESP8266-version: ${version#v}") \
    || die "server not reachable on port $PORT, is '$0 serve' running?"
  case $code in
    200) echo "A sensor on ${version#v} would update to $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["latest"])' "$BIN_DIR/versions")" ;;
    304) echo "A sensor on ${version#v} is up to date" ;;
    *) die "unexpected HTTP $code from the server" ;;
  esac
}

cmd_flash() {
  need_setup
  local version="" erase=false
  for arg in "$@"; do
    case $arg in
      --erase) erase=true ;;
      *) version=${arg#v} ;;
    esac
  done
  [ -n "$version" ] || die "usage: $0 flash <version> [--erase]"
  local bin="$BIN_DIR/$(bin_name "$version")"
  [ -f "$bin" ] || die "$bin not found, see '$0 list'"
  local esptool=(pio pkg exec -p tool-esptoolpy -- esptool.py --chip esp8266)
  [ -n "${UPLOAD_PORT:-}" ] && esptool+=(--port "$UPLOAD_PORT")
  if $erase; then
    "${esptool[@]}" erase_flash
  fi
  "${esptool[@]}" write_flash 0x0 "$bin"
}

command=${1:-help}
shift || true
case $command in
  setup|serve|add|latest|list|check|flash) "cmd_$command" "$@" ;;
  add-release) cmd_add_release "$@" ;;
  *) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//' ;;
esac
