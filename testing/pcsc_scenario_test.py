#!/usr/bin/env python3
"""
PySCard integration test for the SpringCard CCID TCP simulator.

The test waits for the configured reader, waits for card insertion, connects to
the card, exchanges deterministic APDUs, then sends one final APDU that is
expected to return the driver's mute status word while the simulator ejects the
card.
"""

from __future__ import annotations

import argparse
import binascii
import sys
import time
from dataclasses import dataclass


try:
    from smartcard.Exceptions import CardConnectionException, NoCardException
    from smartcard.System import readers
except ImportError as exc:  # pragma: no cover - exercised on hosts without pyscard
    raise SystemExit(
        "PySCard is required. Install it with your distribution package "
        "(for example: sudo apt install python3-pyscard) or with pip."
    ) from exc


@dataclass(frozen=True)
class ApduCase:
    capdu: bytes
    expected: bytes


def hex_bytes(value: str) -> bytes:
    cleaned = "".join(ch for ch in value if ch not in " \t:-_")
    if len(cleaned) % 2:
        raise argparse.ArgumentTypeError(f"odd-length hex string: {value!r}")
    try:
        return binascii.unhexlify(cleaned)
    except binascii.Error as exc:
        raise argparse.ArgumentTypeError(f"invalid hex string: {value!r}") from exc


def parse_apdu_case(value: str) -> ApduCase:
    if "=" not in value:
        raise argparse.ArgumentTypeError("APDU case must use CAPDU=RAPDU")
    capdu, expected = value.split("=", 1)
    return ApduCase(hex_bytes(capdu), hex_bytes(expected))


def wait_until(deadline: float, description: str, probe):
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            result = probe()
            if result:
                return result
        except Exception as exc:  # noqa: BLE001 - printed as diagnostic context
            last_error = exc
        time.sleep(0.25)
    if last_error is not None:
        raise TimeoutError(f"timeout while waiting for {description}: {last_error}") from last_error
    raise TimeoutError(f"timeout while waiting for {description}")


def find_reader(name_part: str, timeout: float):
    deadline = time.monotonic() + timeout

    def probe():
        available = readers()
        for reader in available:
            if name_part.lower() in str(reader).lower():
                return reader
        return None

    return wait_until(deadline, f"reader containing {name_part!r}", probe)


def connect_card(reader, timeout: float):
    deadline = time.monotonic() + timeout

    def probe():
        connection = reader.createConnection()
        connection.connect()
        return connection

    return wait_until(deadline, f"card in {reader}", probe)


def transmit(connection, capdu: bytes) -> bytes:
    data, sw1, sw2 = connection.transmit(list(capdu))
    return bytes(data + [sw1, sw2])


def assert_apdu(connection, case: ApduCase) -> None:
    response = transmit(connection, case.capdu)
    if response != case.expected:
        raise AssertionError(
            f"APDU {case.capdu.hex(' ').upper()} returned "
            f"{response.hex(' ').upper()}, expected {case.expected.hex(' ').upper()}"
        )
    print(f"APDU {case.capdu.hex(' ').upper()} -> {response.hex(' ').upper()}")


def wait_card_removed(reader, timeout: float) -> None:
    deadline = time.monotonic() + timeout

    def probe():
        connection = reader.createConnection()
        try:
            connection.connect()
        except (NoCardException, CardConnectionException):
            return True
        try:
            connection.disconnect()
        except CardConnectionException:
            pass
        return False

    wait_until(deadline, f"card removal from {reader}", probe)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a PySCard scenario against the TCP reader simulator")
    parser.add_argument("--reader-contains", default="PcscOverTcpTest")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--apdu",
        action="append",
        type=parse_apdu_case,
        default=[],
        help="APDU assertion as CAPDU=RAPDU. May be repeated.",
    )
    parser.add_argument("--final-apdu", type=hex_bytes, default=bytes.fromhex("80FE000000"))
    parser.add_argument("--final-rapdu", type=hex_bytes, default=bytes.fromhex("6400"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    apdus = args.apdu or [
        ApduCase(bytes.fromhex("00A4040000"), bytes.fromhex("9000")),
        ApduCase(bytes.fromhex("00CA000000"), bytes.fromhex("112233449000")),
    ]

    reader = find_reader(args.reader_contains, args.timeout)
    print(f"Reader: {reader}")

    connection = connect_card(reader, args.timeout)
    print("Card connected")

    try:
        for case in apdus:
            assert_apdu(connection, case)

        try:
            response = transmit(connection, args.final_apdu)
        except (CardConnectionException, NoCardException) as exc:
            print(f"Final APDU disconnected as expected: {exc}")
        else:
            if response != args.final_rapdu:
                raise AssertionError(
                    f"final APDU returned {response.hex(' ').upper()}, "
                    f"expected {args.final_rapdu.hex(' ').upper()}"
                )
            print(f"Final APDU {args.final_apdu.hex(' ').upper()} -> {response.hex(' ').upper()}")
    finally:
        try:
            connection.disconnect()
        except CardConnectionException:
            pass

    wait_card_removed(reader, args.timeout)
    print("Card removal observed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - command-line diagnostic
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
