#!/bin/sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
TEST_DIR="$ROOT_DIR/testing"
SIMULATOR="$TEST_DIR/ccid_tcp_reader_sim.py"
PCSC_TEST="$TEST_DIR/pcsc_scenario_test.py"
SCENARIO="$TEST_DIR/eject_during_exchange_scenario.txt"
READER_NAME="SpringCard PcscOverTcpTest"
READER_CONF_NAME="springcard-ccid-tcp-test.conf"
SIM_PID=
PCSCD_PID=
PCSCD_WAS_ACTIVE=unknown

if [ "$(id -u)" -eq 0 ]; then
	SUDO=
else
	SUDO=${SUDO:-sudo}
fi

log() {
	printf '%s\n' "$*"
}

need_cmd() {
	if ! command -v "$1" >/dev/null 2>&1; then
		log "Missing command: $1"
		exit 1
	fi
}

cleanup() {
	if [ -n "$PCSCD_PID" ] && kill -0 "$PCSCD_PID" 2>/dev/null; then
		$SUDO kill "$PCSCD_PID" 2>/dev/null || true
		wait "$PCSCD_PID" 2>/dev/null || true
	fi
	if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
		kill "$SIM_PID" 2>/dev/null || true
		wait "$SIM_PID" 2>/dev/null || true
	fi
	if command -v systemctl >/dev/null 2>&1; then
		if [ "$PCSCD_WAS_ACTIVE" = active ]; then
			$SUDO systemctl start pcscd.service >/dev/null 2>&1 || true
			$SUDO systemctl start pcscd.socket >/dev/null 2>&1 || true
		fi
	fi
}
trap cleanup EXIT INT TERM

need_cmd make
need_cmd pkg-config
need_cmd python3
need_cmd pcscd

if [ -n "$SUDO" ] && ! $SUDO -n true >/dev/null 2>&1; then
	log "This test must be run as root, or from a shell where sudo is already authorized."
	log "Try: sudo ./testing/testing.sh"
	exit 1
fi

cd "$ROOT_DIR"

SERIAL_CONFDIR=$(pkg-config libpcsclite --variable=serialconfdir)
USB_DROPDIR=$(pkg-config libpcsclite --variable=usbdropdir)

case "$(uname)" in
	Darwin) BUNDLE_HOST=MacOS ;;
	*) BUNDLE_HOST=$(uname | sed 's,/,_,g') ;;
esac

LIBPATH="$USB_DROPDIR/ifd-springcard-ccid-tcp.bundle/Contents/$BUNDLE_HOST/libspringcard_ccid_tcp.so"
READER_CONF="$SERIAL_CONFDIR/$READER_CONF_NAME"
SIM_LOG="$TEST_DIR/simulator.log"
PCSCD_LOG="$TEST_DIR/pcscd.log"

log "Installing driver with make install"
$SUDO make install

if [ ! -f "$LIBPATH" ]; then
	log "Installed driver library not found: $LIBPATH"
	exit 1
fi

log "Installing active test reader config in $READER_CONF"
$SUDO install -d "$SERIAL_CONFDIR"
TMP_CONF=$(mktemp)
{
	printf 'FRIENDLYNAME      "%s"\n' "$READER_NAME"
	printf 'DEVICENAME        127.0.0.1:3999:00:nokey\n'
	printf 'LIBPATH           %s\n' "$LIBPATH"
	printf 'CHANNELID         0\n'
} > "$TMP_CONF"
$SUDO install -m 0644 "$TMP_CONF" "$READER_CONF"
rm -f "$TMP_CONF"

if command -v systemctl >/dev/null 2>&1; then
	PCSCD_WAS_ACTIVE=$(systemctl is-active pcscd.service 2>/dev/null || true)
	log "Stopping system pcscd service/socket"
	$SUDO systemctl stop pcscd.service >/dev/null 2>&1 || true
	$SUDO systemctl stop pcscd.socket >/dev/null 2>&1 || true
fi
$SUDO pkill pcscd >/dev/null 2>&1 || true

log "Starting reader simulator on 127.0.0.1:3999"
rm -f "$SIM_LOG" "$PCSCD_LOG"
"$SIMULATOR" --host 127.0.0.1 --port 3999 --scenario "$SCENARIO" >"$SIM_LOG" 2>&1 &
SIM_PID=$!
sleep 1
if ! kill -0 "$SIM_PID" 2>/dev/null; then
	log "Simulator failed to start. See $SIM_LOG"
	exit 1
fi

log "Starting pcscd in foreground"
$SUDO pcscd --foreground --debug --apdu >"$PCSCD_LOG" 2>&1 &
PCSCD_PID=$!
sleep 2
if ! kill -0 "$PCSCD_PID" 2>/dev/null; then
	log "pcscd failed to start. See $PCSCD_LOG"
	exit 1
fi

log "Running PySCard scenario"
if ! python3 "$PCSC_TEST" --reader-contains PcscOverTcpTest --timeout 30; then
	log "PySCard scenario failed. Last simulator log lines:"
	tail -n 80 "$SIM_LOG" 2>/dev/null || true
	log "Last pcscd log lines:"
	tail -n 160 "$PCSCD_LOG" 2>/dev/null || true
	exit 1
fi

log "Integration test completed successfully"
