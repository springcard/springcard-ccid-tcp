# SpringCard CCID TCP Reader Simulator

This directory contains a lightweight TCP reader simulator for QA work without
physical SpringCard hardware.

## Full pcscd Integration Test

The integration script installs the driver, creates an active test reader
configuration pointing to `127.0.0.1:3999`, stops the system pcscd instance,
starts the simulator, starts `pcscd --foreground --debug`, then runs a PySCard
scenario.

It needs the usual build dependencies, pcsc-lite tools and PySCard. On
Debian/Ubuntu:

```bash
sudo apt install pcscd pcsc-tools python3-pyscard
```

Run:

```bash
./testing/testing.sh
```

The script writes the active test reader file in the directory returned by:

```bash
pkg-config libpcsclite --variable=serialconfdir
```

This is normally `/etc/reader.conf.d`. Logs are written to
`testing/simulator.log` and `testing/pcscd.log`.

## Manual Simulator Use

Start it with:

```bash
./testing/ccid_tcp_reader_sim.py --host 127.0.0.1 --port 3999 --scenario testing/sample_scenario.txt
```

Then configure the driver with a `DEVICENAME` such as:

```text
DEVICENAME        127.0.0.1:3999:00:nokey
```

Useful options:

```bash
./testing/ccid_tcp_reader_sim.py \
  --vendor-name SpringCard \
  --product-name PcscOverTcpTest \
  --slots 1 \
  --scenario testing/sample_scenario.txt
```

The scenario file is intentionally simple text:

```text
at 1.0 insert 3B00
at 10.0 eject
apdu 00A4040000 -> 9000
apdu * -> 6D00
```

Events are scheduled after the driver sends `SET_CONFIGURATION` with the start
flag. APDU rules are matched in file order. Use `mute eject` as a response to
simulate a silent card and immediate removal. With the current driver this mute
condition is surfaced to PySCard as status word `64 00`, then card removal is
observed separately.
