from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, Optional, Iterable

import serial
from serial.tools import list_ports

# -------------------------------------------------------------------
# Logging (silent by default). Pass verbose=True to Bridge() to enable.
# -------------------------------------------------------------------
_logger = logging.getLogger("can_bridge")
_logger.addHandler(logging.NullHandler())

# -------------------------------------------------------------------
# Protocol
# -------------------------------------------------------------------

MAX_PAYLOAD = 64

class Cmd(IntEnum):
    TARGET_ANGLE       = 0x01
    SET_CURRENT_MA     = 0x02
    SET_SPEED_LIMIT    = 0x03
    SET_PID            = 0x04
    SET_ID             = 0x05
    SET_MICROSTEPS     = 0x06
    SET_STEALTHCHOP    = 0x07
    SET_EXT_MODE       = 0x08
    SET_UNITS          = 0x09
    SET_EXT_ENCODER    = 0x10
    SET_ACCEL_LIMIT    = 0x11
    SET_ENC_INVERT     = 0x0A
    SET_ENABLED        = 0x0B
    SET_STEPS_PER_REV  = 0x0C
    DO_CALIBRATE       = 0x0D
    DO_HOMING          = 0x0E
    SET_ENDSTOP        = 0x0F

# Telemetry emitted by firmware (broadcast)
TELEMETRY_CAN_ID: int = 0x000
TELEMETRY_CMD: int    = 0x01

# Frame header: <HBB> = can_id (u16 LE), cmd (u8), len (u8)
HDR_FMT = "<HBB"
HDR_SIZE = struct.calcsize(HDR_FMT)

# Config & telemetry wire formats (little-endian)
# NOTE: version (u8) is included right after crc32 (u32).
AXIS_CONFIG_WIRE_FMT  = "<I H H B B H H f f f f f H"
AXIS_CONFIG_WIRE_SIZE = struct.calcsize(AXIS_CONFIG_WIRE_FMT)

# DATAPACKET = AxisConfig + four float32: currentSpeed, currentAngle, targetAngle, temperature
DATAPACKET_FMT  = "<I H H B B H H f f f f f H f f f f"
DATAPACKET_SIZE = struct.calcsize(DATAPACKET_FMT)

# Homing parameter payload: <B f B f B>
HOMING_WIRE_FMT  = "<B f B f B"
HOMING_WIRE_SIZE = struct.calcsize(HOMING_WIRE_FMT)

# -------------------------------------------------------------------
# Packing helpers
# -------------------------------------------------------------------

def _pack_u8(v: int)  -> bytes: return struct.pack("<B", v & 0xFF)
def _pack_u16(v: int) -> bytes: return struct.pack("<H", v & 0xFFFF)
def _pack_f32(v: float) -> bytes: return struct.pack("<f", float(v))
def _pack_bool01(b: bool) -> bytes: return struct.pack("<B", 1 if b else 0)

def _make_frame(can_id: int, cmd: int, payload: bytes) -> bytes:
    if not (0 <= can_id <= 0x7FF):
        raise ValueError("can_id must be 11-bit (0..0x7FF)")
    if not (0 <= cmd <= 0xFF):
        raise ValueError("cmd must be 0..255")
    if not (0 <= len(payload) <= MAX_PAYLOAD):
        raise ValueError(f"payload length must be 0..{MAX_PAYLOAD}")
    return struct.pack(HDR_FMT, can_id, cmd, len(payload)) + payload

# -------------------------------------------------------------------
# Models
# -------------------------------------------------------------------

@dataclass
class HomingParams:
    """Parameters sent with DO_HOMING (device interprets units per cfg)."""
    useIN1Trigger: bool = True
    offset: float = 0.0
    activeLow: bool = True
    speed: float = 0.5
    direction: bool = True

def _pack_homing(p: HomingParams) -> bytes:
    return struct.pack(
        HOMING_WIRE_FMT,
        1 if p.useIN1Trigger else 0,
        float(p.offset),
        1 if p.activeLow else 0,
        float(p.speed),
        1 if p.direction else 0,
    )

@dataclass
class AxisFlags:
    encInvert: bool
    dirInvert: bool
    stealthChop: bool
    externalMode: bool
    minTriggered: bool
    maxTriggered: bool
    enableEndstop : bool
    externalEncoder : bool

@dataclass
class AxisConfig:
    crc32: int
    microsteps: int
    stepsPerRev: int
    units: int
    flags: AxisFlags
    encZeroCounts: int
    driver_mA: int
    maxRPS: float
    maxRPS2: float
    Kp: float
    Ki: float
    Kd: float
    canArbId: int

@dataclass
class AxisState:
    config: Optional[AxisConfig] = None
    currentSpeed: Optional[float] = None
    currentAngle: Optional[float] = None
    targetAngle: Optional[float] = None
    temperature : Optional[float] = None
    timestamp: float = field(default_factory=time.time)

# -------------------------------------------------------------------
# Parsing
# -------------------------------------------------------------------

def _parse_axis_config_wire(b: bytes) -> AxisConfig:
    (crc32, microsteps, steps_per_rev, units, flags,
     enc_zero_counts, driver_mA, maxRPS, maxRPS2, Kp, Ki, Kd, canArbId) = struct.unpack(AXIS_CONFIG_WIRE_FMT, b)

    fl = AxisFlags(
        encInvert=bool(flags & 0x01),
        dirInvert=bool(flags & 0x02),
        stealthChop=bool(flags & 0x04),
        externalMode=bool(flags & 0x08),
        minTriggered=bool(flags & 0x10),
        maxTriggered=bool(flags & 0x20),
        enableEndstop=bool(flags & 0x40),
        externalEncoder=bool(flags & 0x80)
    )
    return AxisConfig(
        crc32=crc32,
        microsteps=microsteps,
        stepsPerRev=steps_per_rev,
        units=units,
        flags=fl,
        encZeroCounts=enc_zero_counts,
        driver_mA=driver_mA,
        maxRPS=maxRPS,
        maxRPS2=maxRPS2,
        Kp=Kp, Ki=Ki, Kd=Kd,
        canArbId=canArbId,
    )

def _parse_datapacket(payload: bytes) -> Optional[AxisState]:
    if len(payload) < DATAPACKET_SIZE:
        return None
    fields = struct.unpack(DATAPACKET_FMT, payload[:DATAPACKET_SIZE])
    cfg_bytes = struct.pack(AXIS_CONFIG_WIRE_FMT, *fields[:13])
    cfg = _parse_axis_config_wire(cfg_bytes)
    currentSpeed, currentAngle, targetAngle, temperature = fields[13], fields[14], fields[15], fields[16]
    return AxisState(cfg, currentSpeed, currentAngle, targetAngle, temperature, time.time())

# -------------------------------------------------------------------
# Bridge
# -------------------------------------------------------------------

class Bridge:
    """
    USB bridge for the CAN/CAN-FD serial gateway.
    - You may share one Bridge across many Stepper() instances (one per CAN ID).
    - If port is None, the bridge tries to auto-discover a single suitable USB serial device.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout_s: float = 0.05,
        verbose: bool = False,
        auto_reader: bool = True,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.auto_reader = auto_reader

        # logging
        if verbose:
            _logger.setLevel(logging.DEBUG)
            if not _logger.handlers:
                _logger.addHandler(logging.StreamHandler())

        self._ser: Optional[serial.Serial] = None
        self._reader: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        self._state: Dict[int, AxisState] = {}
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)

    # ----- discovery helpers -----

    @staticmethod
    def find_port(
        *,
        vid: Optional[int] = 0x1209,
        pid: Optional[int] = 0x0011,
        product_substr: Optional[str] = None,
        manufacturer_substr: Optional[str] = None,
    ) -> str:
        """
        Auto-find a single USB-serial port that looks like the CAN-FD bridge.
        You can filter by VID/PID or by product/manufacturer substring.

        Raises:
            RuntimeError if none or multiple candidates are found.
        """
        def matches(p) -> bool:
            if vid is not None and p.vid != vid:
                return False
            if pid is not None and p.pid != pid:
                return False
            if product_substr and (not p.product or product_substr.lower() not in p.product.lower()):
                return False
            if manufacturer_substr and (not p.manufacturer or manufacturer_substr.lower() not in p.manufacturer.lower()):
                return False
            return True

        ports = list(list_ports.comports())
        cands = [p for p in ports if matches(p)]
        # If no filter provided, accept all and require uniqueness.
        if not any([vid, pid, product_substr, manufacturer_substr]):
            cands = ports

        if len(cands) == 0:
            raise RuntimeError("No USB-serial ports found for CAN bridge.")
        if len(cands) > 1:
            desc = ", ".join(f"{p.device}({p.vid:04X}:{p.pid:04X} {p.product or ''})" for p in cands)
            raise RuntimeError(f"Multiple serial ports found; specify one. Candidates: {desc}")
        return cands[0].device

    # ---------- lifecycle ----------

    def open(self) -> None:
        port = self.port or self.find_port()  # auto-detect if not provided
        self._ser = serial.Serial(port, self.baudrate, timeout=self.timeout_s)
        if self.auto_reader:
            self.start_reader()

    def close(self) -> None:
        self.stop_reader()
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def start_reader(self) -> None:
        if self._reader and self._reader.is_alive():
            return
        self._stop_evt.clear()
        self._reader = threading.Thread(target=self._reader_loop, name="can-bridge-reader", daemon=True)
        self._reader.start()

    def stop_reader(self) -> None:
        self._stop_evt.set()
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=1.5)
        self._reader = None

    # ---------- low-level I/O ----------

    def _send(self, can_id: int, cmd: int | Cmd, payload: bytes = b"") -> None:
        if self._ser is None:
            raise RuntimeError("serial not open")
        frame = _make_frame(can_id, int(cmd), payload)
        self._ser.write(frame)
        self._ser.flush()
        _logger.debug("TX id=%03X cmd=%02X len=%d %s", can_id, int(cmd), len(payload), payload.hex())

    def _reader_loop(self) -> None:
        if self._ser is None:
            return
        ser = self._ser

        def read_exact(n: int) -> Optional[bytes]:
            out = bytearray()
            while len(out) < n and not self._stop_evt.is_set():
                chunk = ser.read(n - len(out))
                if not chunk:
                    continue  # timeout
                out.extend(chunk)
            return bytes(out) if len(out) == n else None

        while not self._stop_evt.is_set():
            try:
                hdr = read_exact(HDR_SIZE)
                if hdr is None:
                    continue
                can_id, cmd, length = struct.unpack(HDR_FMT, hdr)
                if length > MAX_PAYLOAD:
                    _logger.debug("RX bad length=%d (drop)", length)
                    _ = ser.read(length)  # drain
                    continue
                payload = read_exact(length) or b""
                _logger.debug("RX id=%03X cmd=%02X len=%d %s", can_id, cmd, len(payload), payload.hex())

                # telemetry fanout
                if can_id == TELEMETRY_CAN_ID and cmd == TELEMETRY_CMD:
                    pkt = _parse_datapacket(payload)
                    if pkt and pkt.config:
                        with self._lock:
                            self._state[pkt.config.canArbId] = pkt
                            self._cond.notify_all()

            except serial.SerialException as e:
                _logger.debug("serial error: %s", e)
                break
            except Exception as e:
                _logger.debug("reader error: %s", e)

    # ---------- command helpers (SET) ----------

    def set_target_angle(self, can_id: int, angle: float) -> None:
        self._send(can_id, Cmd.TARGET_ANGLE, _pack_f32(angle))

    def set_current_ma(self, can_id: int, ma: int) -> None:
        self._send(can_id, Cmd.SET_CURRENT_MA, _pack_u16(ma))

    def set_speed_limit_rps(self, can_id: int, rps: float) -> None:
        self._send(can_id, Cmd.SET_SPEED_LIMIT, _pack_f32(rps))
    
    def set_accel_limit_rps2(self, can_id: int, rps2: float) -> None:
        self._send(can_id, Cmd.SET_ACCEL_LIMIT, _pack_f32(rps2))

    def set_pid(self, can_id: int, kp: float, ki: float, kd: float) -> None:
        self._send(can_id, Cmd.SET_PID, _pack_f32(kp) + _pack_f32(ki) + _pack_f32(kd))

    def set_can_id(self, can_id: int, new_id: int) -> None:
        self._send(can_id, Cmd.SET_ID, _pack_u16(new_id & 0x7FF))

    def set_microsteps(self, can_id: int, microsteps: int) -> None:
        self._send(can_id, Cmd.SET_MICROSTEPS, _pack_u16(microsteps))

    def set_stealthchop(self, can_id: int, enable: bool) -> None:
        self._send(can_id, Cmd.SET_STEALTHCHOP, _pack_bool01(enable))

    def set_external_mode(self, can_id: int, enable: bool) -> None:
        self._send(can_id, Cmd.SET_EXT_MODE, _pack_bool01(enable))

    def set_external_encoder(self, can_id: int, enable: bool) -> None:
        self._send(can_id, Cmd.SET_EXT_ENCODER, _pack_bool01(enable))

    def set_units_degrees(self, can_id: int, use_degrees: bool) -> None:
        self._send(can_id, Cmd.SET_UNITS, _pack_u8(1 if use_degrees else 0))

    def set_encoder_invert(self, can_id: int, enable: bool) -> None:
        self._send(can_id, Cmd.SET_ENC_INVERT, _pack_bool01(enable))

    def enable_motor(self, can_id: int, enable: bool) -> None:
        self._send(can_id, Cmd.SET_ENABLED, _pack_bool01(enable))

    def set_steps_per_rev(self, can_id: int, steps_per_rev: int) -> None:
        self._send(can_id, Cmd.SET_STEPS_PER_REV, _pack_u16(steps_per_rev))

    def do_calibrate(self, can_id: int) -> None:
        self._send(can_id, Cmd.DO_CALIBRATE, b"")

    def do_homing(self, can_id: int, params: HomingParams) -> None:
        self._send(can_id, Cmd.DO_HOMING, _pack_homing(params))

    # ---------- telemetry access (GET) ----------

    def _wait_state(self, can_id: int, timeout_s: Optional[float]) -> Optional[AxisState]:
        deadline = None if timeout_s is None else (time.time() + timeout_s)
        with self._lock:
            if timeout_s is None and can_id in self._state:
                return self._state[can_id]
            while True:
                if can_id in self._state:
                    return self._state[can_id]
                if deadline is not None:
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        return None
                    self._cond.wait(timeout=remaining)
                else:
                    self._cond.wait()

    def get_current_angle(self, can_id: int, timeout_s: Optional[float] = None) -> Optional[float]:
        st = self._wait_state(can_id, timeout_s)
        return None if st is None else st.currentAngle

    def get_target_angle(self, can_id: int, timeout_s: Optional[float] = None) -> Optional[float]:
        st = self._wait_state(can_id, timeout_s)
        return None if st is None else st.targetAngle

    def get_current_speed(self, can_id: int, timeout_s: Optional[float] = None) -> Optional[float]:
        st = self._wait_state(can_id, timeout_s)
        return None if st is None else st.currentSpeed

    def get_axis_state(self, can_id: int, timeout_s: Optional[float] = None) -> Optional[AxisState]:
        return self._wait_state(can_id, timeout_s)

# -------------------------------------------------------------------
# Per-motor convenience proxy
# -------------------------------------------------------------------

class Stepper:
    """
    Convenience wrapper around Bridge for a single motor (CAN ID).
    You can create many Stepper() objects sharing one Bridge.
    """

    def __init__(self, bridge: Bridge, can_id: int):
        if not (0 <= can_id <= 0x7FF):
            raise ValueError("can_id must be 11-bit (0..0x7FF)")
        self.bridge = bridge
        self.can_id = can_id

    # ---- setters ----
    def set_target_angle(self, angle: float) -> None:
        self.bridge.set_target_angle(self.can_id, angle)

    def set_current_ma(self, ma: int) -> None:
        self.bridge.set_current_ma(self.can_id, ma)

    def set_speed_limit_rps(self, rps: float) -> None:
        self.bridge.set_speed_limit_rps(self.can_id, rps)

    def set_accel_limit_rps2(self, rps2: float) -> None:
        self.bridge.set_accel_limit_rps2(self.can_id, rps2)

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        self.bridge.set_pid(self.can_id, kp, ki, kd)

    def set_can_id(self, new_id: int) -> None:
        self.bridge.set_can_id(self.can_id, new_id)
        self.can_id = (new_id & 0x7FF)

    def set_microsteps(self, microsteps: int) -> None:
        self.bridge.set_microsteps(self.can_id, microsteps)

    def set_stealthchop(self, enable: bool) -> None:
        self.bridge.set_stealthchop(self.can_id, enable)

    def set_external_mode(self, enable: bool) -> None:
        self.bridge.set_external_mode(self.can_id, enable)

    def set_external_encoder(self, enable: bool) -> None:
        self.bridge.set_external_encoder(self.can_id, enable)

    def set_units_degrees(self, use_degrees: bool) -> None:
        self.bridge.set_units_degrees(self.can_id, use_degrees)

    def set_encoder_invert(self, enable: bool) -> None:
        self.bridge.set_encoder_invert(self.can_id, enable)

    def enable_motor(self, enable: bool) -> None:
        self.bridge.enable_motor(self.can_id, enable)

    def set_steps_per_rev(self, steps_per_rev: int) -> None:
        self.bridge.set_steps_per_rev(self.can_id, steps_per_rev)

    def do_calibrate(self) -> None:
        self.bridge.do_calibrate(self.can_id)

    def do_homing(self, params: HomingParams) -> None:
        self.bridge.do_homing(self.can_id, params)

    # ---- getters ----
    def get_axis_state(self, timeout_s: Optional[float] = None) -> Optional[AxisState]:
        return self.bridge.get_axis_state(self.can_id, timeout_s)

    def get_current_angle(self, timeout_s: Optional[float] = None) -> Optional[float]:
        return self.bridge.get_current_angle(self.can_id, timeout_s)

    def get_target_angle(self, timeout_s: Optional[float] = None) -> Optional[float]:
        return self.bridge.get_target_angle(self.can_id, timeout_s)

    def get_current_speed(self, timeout_s: Optional[float] = None) -> Optional[float]:
        return self.bridge.get_current_speed(self.can_id, timeout_s)