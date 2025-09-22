import serial, struct, threading, queue, time

STX = 0xAA

class Cmd:
    TARGET_ANGLE      = 0x01
    SET_CURRENT_MA    = 0x02
    SET_SPEED_LIMIT   = 0x03
    SET_PID           = 0x04
    SET_ID            = 0x05  # u16 in firmware (11-bit used)
    SET_MICROSTEPS    = 0x06  # u16
    SET_STEALTHCHOP   = 0x07  # u8 0/1
    SET_EXT_MODE      = 0x08  # u8 0/1 (bool)
    SET_UNITS         = 0x09  # u8: 0=radians, 1=degrees
    SET_ENC_INVERT    = 0x0A  # u8 0/1
    SET_ENABLED       = 0x0B  # u8 0/1
    SET_STEPS_PER_REV = 0x0C  # u16
    DO_CALIBRATE      = 0x0D  # no payload

class StepperLink:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=timeout, exclusive=True)
        time.sleep(0.05)
        self._q: "queue.Queue[tuple[int, bytes]]" = queue.Queue(maxsize=256)
        self._stop = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, name="stepper-rx", daemon=True)
        self._rx.start()

    def close(self) -> None:
        self._stop.set()
        try: self._rx.join(0.5)
        except Exception: pass
        try: self._ser.close()
        except Exception: pass

    @staticmethod
    def _crc16_ibm(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
        return crc & 0xFFFF

    def _pack(self, cmd: int, payload: bytes) -> bytes:
        head = bytes([STX, cmd & 0xFF, len(payload) & 0xFF]) + payload
        return head + struct.pack('<H', self._crc16_ibm(head))

    def _rx_loop(self) -> None:
        r = self._ser.read
        while not self._stop.is_set():
            b = r(1)
            if not b or b[0] != STX:
                continue
            header = r(2)
            if len(header) != 2:
                continue
            cmd, ln = header[0], header[1]
            pay = r(ln)
            crc = r(2)
            if len(pay) != ln or len(crc) != 2:
                continue
            calc = self._crc16_ibm(bytes([STX, cmd, ln]) + pay)
            (rx_crc,) = struct.unpack('<H', crc)
            if rx_crc != calc:
                continue
            try:
                self._q.put_nowait((cmd, pay))
            except queue.Full:
                try: self._q.get_nowait()
                except queue.Empty: pass
                try: self._q.put_nowait((cmd, pay))
                except queue.Full: pass

    def send(self, cmd: int, payload: bytes = b'') -> None:
        self._ser.write(self._pack(cmd, payload))
        self._ser.flush()

    def read(self, timeout: float | None = None) -> tuple[int, bytes] | None:
        try: return self._q.get(timeout=timeout)
        except queue.Empty: return None

    # ---- typed writers ----
    def set_target_angle(self, angle: float) -> None:
        self.send(Cmd.TARGET_ANGLE, struct.pack('<f', float(angle)))

    def set_current_ma(self, ma: int) -> None:
        self.send(Cmd.SET_CURRENT_MA, struct.pack('<H', int(ma)))

    def set_speed_limit_rps(self, rps: float) -> None:
        self.send(Cmd.SET_SPEED_LIMIT, struct.pack('<f', float(rps)))

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        self.send(Cmd.SET_PID, struct.pack('<fff', float(kp), float(ki), float(kd)))

    def set_id(self, arb_id: int) -> None:
        self.send(Cmd.SET_ID, struct.pack('<H', int(arb_id) & 0x7FF))

    def set_microsteps(self, microsteps: int) -> None:
        self.send(Cmd.SET_MICROSTEPS, struct.pack('<H', int(microsteps)))

    def set_stealthchop(self, enable: bool) -> None:
        self.send(Cmd.SET_STEALTHCHOP, struct.pack('<B', 1 if enable else 0))

    def set_ext_mode(self, enable: bool) -> None:
        self.send(Cmd.SET_EXT_MODE, struct.pack('<B', 1 if enable else 0))

    def set_units(self, units: int) -> None:  # 0=radians, 1=degrees
        self.send(Cmd.SET_UNITS, struct.pack('<B', int(units) & 0xFF))

    def set_encoder_invert(self, invert: bool) -> None:
        self.send(Cmd.SET_ENC_INVERT, struct.pack('<B', 1 if invert else 0))

    def set_enabled(self, enable: bool) -> None:
        self.send(Cmd.SET_ENABLED, struct.pack('<B', 1 if enable else 0))

    def set_steps_per_rev(self, steps: int) -> None:
        self.send(Cmd.SET_STEPS_PER_REV, struct.pack('<H', int(steps)))

    def do_calibrate(self) -> None:
        self.send(Cmd.DO_CALIBRATE, b'')

    # ---- telemetry helpers ----
    @staticmethod
    def parse_axis_config_wire(b: bytes) -> dict:
        if len(b) < 34:
            raise ValueError('AxisConfigWire too short')
        (crc32,
         microsteps,
         stepsPerRev,
         units,
         flags,
         encZeroCounts,
         driver_mA,
         maxRPS,
         Kp,
         Ki,
         Kd,
         flags2,
         canArbId) = struct.unpack('<IHHBBHHffffHH', b[:34])
        return {
            'crc32': crc32,
            'microsteps': microsteps,
            'stepsPerRev': stepsPerRev,
            'units': units,                 # 0=radians, 1=degrees
            'encInvert': bool(flags & 0x01),
            'dirInvert': bool(flags & 0x02),
            'stealthChop': bool(flags & 0x04),
            'externalMode': bool(flags & 0x08),
            'encZeroCounts': encZeroCounts,
            'driver_mA': driver_mA,
            'maxRPS': maxRPS,
            'Kp': Kp,
            'Ki': Ki,
            'Kd': Kd,
            'flags_raw': flags,
            'flags2': flags2,
            'canArbId': canArbId & 0x7FF,
        }

    @staticmethod
    def parse_datapacket(payload: bytes) -> dict:
        if len(payload) < 58:
            raise ValueError('DataPacket too short')
        cfg = StepperLink.parse_axis_config_wire(payload[:34])
        currentSpeed, currentAngle, targetAngle = struct.unpack('<ddd', payload[34:34+24])
        return {
            'config': cfg,
            'currentSpeed': currentSpeed,
            'currentAngle': currentAngle,
            'targetAngle': targetAngle,
        }