#!/usr/bin/env python3
"""
Small SpringCard CCID-over-TCP reader simulator.

The simulator is intentionally conservative: it implements the discovery and
minimum CCID command set needed to let pcscd load the driver, see card
presence, power a pseudo-card and exchange simple APDUs.

Scenario file format:

    # Events are scheduled N seconds after SET_CONFIGURATION starts the reader.
    at 1.0 insert 3B00
    at 5.0 eject

    # APDU mappings are hexadecimal. Spaces are ignored.
    apdu 00A4040000 -> 9000
    apdu 00 CA 00 00 00 -> 1122339000

    # Fault injection.
    apdu 80FE000000 -> mute eject
    apdu * -> 6D00
"""

from __future__ import annotations

import argparse
import binascii
import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path


EP_CONTROL_OUT = 0x00
EP_CONTROL_IN = 0x80
EP_BULK_OUT = 0x02
EP_BULK_IN = 0x81
EP_INTERRUPT_IN = 0x83

GET_STATUS = 0x00
GET_DESCRIPTOR = 0x06
SET_CONFIGURATION = 0x09

PC_TO_RDR_ICC_POWER_ON = 0x62
PC_TO_RDR_ICC_POWER_OFF = 0x63
PC_TO_RDR_GET_SLOT_STATUS = 0x65
PC_TO_RDR_XFR_BLOCK = 0x6F
PC_TO_RDR_GET_PARAMETERS = 0x6C
PC_TO_RDR_RESET_PARAMETERS = 0x6D
PC_TO_RDR_SET_PARAMETERS = 0x61
PC_TO_RDR_ESCAPE = 0x6B

RDR_TO_PC_DATA_BLOCK = 0x80
RDR_TO_PC_SLOT_STATUS = 0x81
RDR_TO_PC_PARAMETERS = 0x82
RDR_TO_PC_ESCAPE = 0x83
RDR_TO_PC_UNSUPPORTED = 0x00

RDR_TO_PC_NOTIFY_SLOT_CHANGE = 0x50

CCID_ICC_PRESENT_ACTIVE = 0x00
CCID_ICC_PRESENT_INACTIVE = 0x01
CCID_ICC_ABSENT = 0x02
CCID_COMMAND_FAILED = 0x40

CCID_ERROR_CMD_NOT_SUPPORTED = 0x00
CCID_ERROR_ICC_MUTE = 0xFE
CCID_ERROR_XFR_TIMEOUT = 0xF0

CONTROL_STATUS_OK = 0x00
CONTROL_STATUS_DENIED = 0xFD
CONTROL_STATUS_PROTOCOL_ERROR = 0xFF

DEFAULT_ATR = bytes.fromhex("3B00")
DEFAULT_SERIAL = "01020304"


@dataclass(frozen=True)
class ScenarioEvent:
    delay: float
    action: str
    slot: int = 0
    atr: bytes = DEFAULT_ATR


@dataclass(frozen=True)
class ApduRule:
    capdu: bytes | None
    response: bytes | None
    mute: bool = False
    eject: bool = False


@dataclass
class Scenario:
    events: list[ScenarioEvent] = field(default_factory=list)
    apdu_rules: list[ApduRule] = field(default_factory=list)

    def response_for(self, capdu: bytes) -> ApduRule:
        for rule in self.apdu_rules:
            if rule.capdu is None or rule.capdu == capdu:
                return rule
        return ApduRule(capdu=None, response=bytes.fromhex("9000"))


@dataclass
class SlotState:
    present: bool = False
    powered: bool = False
    atr: bytes = DEFAULT_ATR

    @property
    def ccid_status(self) -> int:
        if not self.present:
            return CCID_ICC_ABSENT
        if not self.powered:
            return CCID_ICC_PRESENT_INACTIVE
        return CCID_ICC_PRESENT_ACTIVE


@dataclass
class SimulatorConfig:
    host: str
    port: int
    vendor_name: str
    product_name: str
    serial_number: str
    slots: int
    vendor_id: int
    product_id: int
    version: int
    scenario: Scenario


def hex_bytes(text: str) -> bytes:
    cleaned = "".join(ch for ch in text if ch not in " \t:-_")
    if not cleaned:
        return b""
    if len(cleaned) % 2:
        raise ValueError(f"odd-length hex string: {text!r}")
    return binascii.unhexlify(cleaned)


def parse_scenario(path: Path | None) -> Scenario:
    scenario = Scenario()
    if path is None:
        return scenario
    if not path.is_file():
        raise ValueError(f"scenario file not found: {path}")

    for lineno, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        keyword = parts[0].lower()
        try:
            if keyword == "at" and len(parts) >= 3:
                delay = float(parts[1])
                action = parts[2].lower()
                slot = 0
                atr = DEFAULT_ATR
                rest = parts[3:]
                if rest and rest[0].lower().startswith("slot="):
                    slot = int(rest.pop(0).split("=", 1)[1], 0)
                if action == "insert":
                    if not rest:
                        raise ValueError("insert requires an ATR")
                    atr = hex_bytes("".join(rest))
                elif action != "eject":
                    raise ValueError(f"unknown event action {action!r}")
                scenario.events.append(ScenarioEvent(delay, action, slot, atr))
            elif keyword == "apdu":
                if "->" not in parts:
                    raise ValueError("APDU rule requires '->'")
                arrow = parts.index("->")
                left = "".join(parts[1:arrow])
                right = [part.lower() for part in parts[arrow + 1 :]]
                if not left:
                    raise ValueError("APDU rule requires a C-APDU")
                capdu = None if left == "*" else hex_bytes(left)
                mute = "mute" in right
                eject = "eject" in right
                response_tokens = [part for part in right if part not in {"mute", "eject"}]
                response = hex_bytes("".join(response_tokens)) if response_tokens else None
                scenario.apdu_rules.append(ApduRule(capdu, response, mute, eject))
            else:
                raise ValueError(f"unknown scenario directive {keyword!r}")
        except ValueError as exc:
            raise ValueError(f"{path}:{lineno}: {exc}") from exc

    scenario.events.sort(key=lambda event: event.delay)
    return scenario


def le32(value: int) -> bytes:
    return struct.pack("<I", value)


def usb_string_descriptor(value: str) -> bytes:
    encoded = value.encode("utf-16le")
    length = len(encoded) + 2
    if length > 255:
        encoded = encoded[:253]
        length = len(encoded) + 2
    return bytes([length, 0x03]) + encoded


def network_frame(endpoint: int, message_type: int, payload: bytes, suffix: bytes = b"\x00" * 5) -> bytes:
    return bytes([endpoint, message_type]) + le32(len(payload)) + suffix[:5].ljust(5, b"\x00") + payload


def ccid_frame(
    endpoint: int,
    message_type: int,
    slot: int,
    seq: int,
    payload: bytes = b"",
    status: int = CCID_ICC_PRESENT_ACTIVE,
    error: int = 0x00,
    chain_or_param: int = 0x00,
) -> bytes:
    header = bytes([message_type]) + le32(len(payload)) + bytes(
        [slot & 0xFF, seq & 0xFF, status & 0xFF, error & 0xFF, chain_or_param & 0xFF]
    )
    return bytes([endpoint]) + header + payload


def unsupported(seq: int, slot: int = 0) -> bytes:
    return ccid_frame(
        EP_BULK_IN,
        RDR_TO_PC_UNSUPPORTED,
        slot,
        seq,
        status=CCID_COMMAND_FAILED | CCID_ICC_ABSENT,
        error=CCID_ERROR_CMD_NOT_SUPPORTED,
    )


def build_device_descriptor(config: SimulatorConfig) -> bytes:
    return bytes(
        [
            18,
            0x01,
            0x00,
            0x02,
            0x00,
            0x00,
            0x00,
            64,
            (config.vendor_id >> 8) & 0xFF,
            config.vendor_id & 0xFF,
            (config.product_id >> 8) & 0xFF,
            config.product_id & 0xFF,
            (config.version >> 8) & 0xFF,
            config.version & 0xFF,
            1,
            2,
            3,
            1,
        ]
    )


def build_configuration_descriptor(config: SimulatorConfig) -> bytes:
    max_slot_index = config.slots - 1
    ccid = bytearray(54)
    ccid[0] = 0x36
    ccid[1] = 0x21
    ccid[2:4] = bytes([0x10, 0x01])
    ccid[4] = max_slot_index
    ccid[5] = 0x07
    ccid[6:10] = le32(0x00000003)
    ccid[10:14] = le32(4000)
    ccid[14:18] = le32(4000)
    ccid[18] = 0x00
    ccid[19:23] = le32(10752)
    ccid[23:27] = le32(250000)
    ccid[27] = 0x00
    ccid[28:32] = le32(254)
    ccid[32:36] = le32(0)
    ccid[36:40] = le32(0)
    ccid[40:44] = le32(0x0004047E)
    ccid[44:48] = le32(65554)
    ccid[48] = 0xFF
    ccid[49] = 0xFF
    ccid[50:52] = bytes([0, 0])
    ccid[52] = 0x00
    ccid[53] = 0x01

    total_length = 9 + 9 + len(ccid) + 7 + 7 + 7
    configuration = bytes([9, 2]) + struct.pack("<H", total_length) + bytes([1, 1, 4, 0x80, 50])
    interface = bytes([9, 4, 0, 0, 3, 0x0B, 0, 0, 0])
    bulk_in = bytes([7, 5, EP_BULK_IN, 2, 0x40, 0x00, 0])
    bulk_out = bytes([7, 5, EP_BULK_OUT, 2, 0x40, 0x00, 0])
    interrupt = bytes([7, 5, EP_INTERRUPT_IN, 3, 0x10, 0x00, 0])
    return configuration + interface + bytes(ccid) + bulk_in + bulk_out + interrupt


class ReaderSession:
    def __init__(self, conn: socket.socket, address: tuple[str, int], config: SimulatorConfig):
        self.conn = conn
        self.address = address
        self.config = config
        self.slots = [SlotState() for _ in range(config.slots)]
        self.running = False
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.scheduler_thread: threading.Thread | None = None

    def log(self, message: str, *args: object) -> None:
        logging.info("[%s:%s] " + message, self.address[0], self.address[1], *args)

    def run(self) -> None:
        self.log("connected")
        try:
            while not self.closed.is_set():
                frame = self.recv_frame()
                if frame is None:
                    break
                self.handle_frame(frame)
        except (ConnectionError, OSError) as exc:
            self.log("connection ended: %s", exc)
        finally:
            self.closed.set()
            self.conn.close()
            self.log("disconnected")

    def recv_exact(self, length: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < length:
            data = self.conn.recv(length - len(chunks))
            if not data:
                raise ConnectionError("peer closed the connection")
            chunks.extend(data)
        return bytes(chunks)

    def recv_frame(self) -> bytes | None:
        try:
            header = self.recv_exact(11)
        except ConnectionError:
            return None
        payload_len = struct.unpack("<I", header[2:6])[0]
        if payload_len > 65554:
            self.send_control_status(CONTROL_STATUS_PROTOCOL_ERROR)
            raise ConnectionError(f"oversized frame: {payload_len}")
        return header + self.recv_exact(payload_len)

    def send(self, data: bytes) -> None:
        self.conn.sendall(data)

    def handle_frame(self, frame: bytes) -> None:
        endpoint = frame[0]
        message_type = frame[1]
        if endpoint == EP_CONTROL_OUT:
            self.handle_control(frame)
        elif endpoint == EP_BULK_OUT:
            if not self.running:
                self.send_control_status(CONTROL_STATUS_DENIED)
                return
            self.handle_bulk(message_type, frame)
        else:
            self.send_control_status(CONTROL_STATUS_PROTOCOL_ERROR)

    def handle_control(self, frame: bytes) -> None:
        message_type = frame[1]
        if message_type == GET_STATUS:
            self.send_control_status(CONTROL_STATUS_OK)
        elif message_type == GET_DESCRIPTOR:
            self.handle_get_descriptor(frame)
        elif message_type == SET_CONFIGURATION:
            self.handle_set_configuration(frame)
        else:
            self.send_control_status(CONTROL_STATUS_PROTOCOL_ERROR)

    def send_control_status(self, status: int) -> None:
        self.send(network_frame(EP_CONTROL_IN, GET_STATUS, b"", bytes([0, 0, 0, 0, status])))

    def handle_get_descriptor(self, frame: bytes) -> None:
        descriptor_type = frame[6]
        descriptor_index = frame[7]
        if descriptor_type == 1 and descriptor_index == 0:
            payload = build_device_descriptor(self.config)
        elif descriptor_type == 2 and descriptor_index == 0:
            payload = build_configuration_descriptor(self.config)
        elif descriptor_type == 3:
            strings = {
                1: self.config.vendor_name,
                2: self.config.product_name,
                3: self.config.serial_number,
                4: "CCID",
            }
            payload = usb_string_descriptor(strings.get(descriptor_index, ""))
        else:
            payload = b""
        suffix = bytes([descriptor_type, descriptor_index, 0, 0, 0])
        self.send(network_frame(EP_CONTROL_IN, GET_DESCRIPTOR, payload, suffix))

    def handle_set_configuration(self, frame: bytes) -> None:
        start = frame[6]
        option = frame[10]
        with self.lock:
            self.running = start != 0
            if not self.running:
                for slot in self.slots:
                    slot.powered = False
        self.log("SET_CONFIGURATION running=%s option=0x%02X", self.running, option)
        if self.running and self.scheduler_thread is None:
            self.scheduler_thread = threading.Thread(target=self.run_scenario_events, daemon=True)
            self.scheduler_thread.start()
        suffix = bytes([0, start, 0, 0, start])
        self.send(network_frame(EP_CONTROL_IN, SET_CONFIGURATION, b"", suffix))

    def handle_bulk(self, message_type: int, frame: bytes) -> None:
        payload = frame[11:]
        slot = frame[6]
        seq = frame[7]
        if slot >= len(self.slots):
            self.send(unsupported(seq, 0))
            return

        if message_type == PC_TO_RDR_GET_SLOT_STATUS:
            self.send_slot_status(slot, seq)
        elif message_type == PC_TO_RDR_ICC_POWER_ON:
            self.power_on(slot, seq)
        elif message_type == PC_TO_RDR_ICC_POWER_OFF:
            self.power_off(slot, seq)
        elif message_type == PC_TO_RDR_XFR_BLOCK:
            self.exchange_apdu(slot, seq, payload)
        elif message_type in (PC_TO_RDR_GET_PARAMETERS, PC_TO_RDR_RESET_PARAMETERS):
            self.send_parameters(slot, seq, protocol=0)
        elif message_type == PC_TO_RDR_SET_PARAMETERS:
            protocol = frame[8]
            self.send_parameters(slot, seq, protocol=protocol)
        elif message_type == PC_TO_RDR_ESCAPE:
            self.send(ccid_frame(EP_BULK_IN, RDR_TO_PC_ESCAPE, slot, seq, payload=b""))
        else:
            self.send(unsupported(seq, slot))

    def send_slot_status(self, slot_index: int, seq: int) -> None:
        with self.lock:
            status = self.slots[slot_index].ccid_status
        self.send(ccid_frame(EP_BULK_IN, RDR_TO_PC_SLOT_STATUS, slot_index, seq, status=status))

    def power_on(self, slot_index: int, seq: int) -> None:
        with self.lock:
            slot = self.slots[slot_index]
            if not slot.present:
                self.send(
                    ccid_frame(
                        EP_BULK_IN,
                        RDR_TO_PC_DATA_BLOCK,
                        slot_index,
                        seq,
                        status=CCID_COMMAND_FAILED | CCID_ICC_ABSENT,
                        error=CCID_ERROR_ICC_MUTE,
                    )
                )
                return
            slot.powered = True
            atr = slot.atr
        self.send(ccid_frame(EP_BULK_IN, RDR_TO_PC_DATA_BLOCK, slot_index, seq, payload=atr))

    def power_off(self, slot_index: int, seq: int) -> None:
        with self.lock:
            self.slots[slot_index].powered = False
            status = self.slots[slot_index].ccid_status
        self.send(ccid_frame(EP_BULK_IN, RDR_TO_PC_SLOT_STATUS, slot_index, seq, status=status))

    def exchange_apdu(self, slot_index: int, seq: int, capdu: bytes) -> None:
        with self.lock:
            slot = self.slots[slot_index]
            if not slot.present:
                self.send(
                    ccid_frame(
                        EP_BULK_IN,
                        RDR_TO_PC_DATA_BLOCK,
                        slot_index,
                        seq,
                        status=CCID_COMMAND_FAILED | CCID_ICC_ABSENT,
                        error=CCID_ERROR_ICC_MUTE,
                    )
                )
                return
            if not slot.powered:
                slot.powered = True
        rule = self.config.scenario.response_for(capdu)
        if rule.mute:
            if rule.eject:
                self.set_card(slot_index, present=False)
            self.send(
                ccid_frame(
                    EP_BULK_IN,
                    RDR_TO_PC_DATA_BLOCK,
                    slot_index,
                    seq,
                    status=CCID_COMMAND_FAILED | CCID_ICC_PRESENT_ACTIVE,
                    error=CCID_ERROR_XFR_TIMEOUT,
                )
            )
            return
        if rule.eject:
            self.set_card(slot_index, present=False)
        response = rule.response if rule.response is not None else bytes.fromhex("9000")
        self.log("APDU %s -> %s", capdu.hex(" ").upper(), response.hex(" ").upper())
        self.send(ccid_frame(EP_BULK_IN, RDR_TO_PC_DATA_BLOCK, slot_index, seq, payload=response))

    def send_parameters(self, slot_index: int, seq: int, protocol: int) -> None:
        if protocol == 1:
            payload = bytes([0x11, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00])
        else:
            payload = bytes([0x11, 0x00, 0x00, 0x0A, 0x00])
        self.send(ccid_frame(EP_BULK_IN, RDR_TO_PC_PARAMETERS, slot_index, seq, payload=payload))

    def set_card(self, slot_index: int, present: bool, atr: bytes = DEFAULT_ATR) -> None:
        with self.lock:
            slot = self.slots[slot_index]
            slot.present = present
            slot.powered = False
            if present:
                slot.atr = atr
        action = "inserted" if present else "removed"
        self.log("slot %d card %s", slot_index, action)
        self.send_slot_notification(slot_index, inserted=present)

    def send_slot_notification(self, slot_index: int, inserted: bool) -> None:
        if not self.running:
            return
        bits = 0
        for index, slot in enumerate(self.slots[:4]):
            if index == slot_index:
                value = 0b11 if inserted else 0b10
            else:
                value = 0b01 if slot.present else 0b00
            bits |= value << (2 * index)
        payload = bytes([bits])
        frame = network_frame(EP_INTERRUPT_IN, RDR_TO_PC_NOTIFY_SLOT_CHANGE, payload)
        try:
            self.send(frame)
        except OSError:
            self.closed.set()

    def run_scenario_events(self) -> None:
        start = time.monotonic()
        for event in self.config.scenario.events:
            wait = event.delay - (time.monotonic() - start)
            if wait > 0 and self.closed.wait(wait):
                return
            if event.slot >= len(self.slots):
                self.log("ignoring scenario event for missing slot %d", event.slot)
                continue
            if event.action == "insert":
                self.set_card(event.slot, present=True, atr=event.atr)
            elif event.action == "eject":
                self.set_card(event.slot, present=False)


class ReaderServer:
    def __init__(self, config: SimulatorConfig):
        self.config = config
        self.stop_event = threading.Event()

    def serve_forever(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.config.host, self.config.port))
            server.listen()
            server.settimeout(0.5)
            logging.info("listening on %s:%d", self.config.host, self.config.port)
            while not self.stop_event.is_set():
                try:
                    conn, address = server.accept()
                except socket.timeout:
                    continue
                session = ReaderSession(conn, address, self.config)
                threading.Thread(target=session.run, daemon=True).start()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SpringCard CCID-over-TCP reader simulator")
    parser.add_argument("--host", default="0.0.0.0", help="address to bind, default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=3999, help="TCP port to listen on, default: 3999")
    parser.add_argument("--vendor-name", default="SpringCard", help="USB string descriptor 1")
    parser.add_argument("--product-name", default="PcscOverTcpTest", help="USB string descriptor 2")
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL, help="USB string descriptor 3")
    parser.add_argument("--slots", type=int, default=1, choices=range(1, 5), metavar="1..4")
    parser.add_argument("--vendor-id", type=lambda value: int(value, 0), default=0x1C34)
    parser.add_argument("--product-id", type=lambda value: int(value, 0), default=0xFFFF)
    parser.add_argument("--version", type=lambda value: int(value, 0), default=0x0100)
    parser.add_argument("--scenario", type=Path, help="TXT scenario file")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        scenario = parse_scenario(args.scenario)
    except ValueError as exc:
        logging.error("%s", exc)
        return 2
    config = SimulatorConfig(
        host=args.host,
        port=args.port,
        vendor_name=args.vendor_name,
        product_name=args.product_name,
        serial_number=args.serial_number,
        slots=args.slots,
        vendor_id=args.vendor_id,
        product_id=args.product_id,
        version=args.version,
        scenario=scenario,
    )
    server = ReaderServer(config)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("stopping")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
