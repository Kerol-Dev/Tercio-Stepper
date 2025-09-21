import argparse, struct, sys, serial
from dataclasses import dataclass
from typing import Optional

HDR_SIZE = 4   # [ID_L][ID_H][CMD][LEN]
CMD_TELEM = 0x01

# ---- AxisConfigWire (exact 32B, matches your C++) ----
AXIS_FMT = "<I H B B H H f f f f H H"
AXIS_SIZE = struct.calcsize(AXIS_FMT)  # 32

@dataclass
class AxisConfigWire:
    crc32: int
    microsteps: int
    units: int
    flags_bools: int
    encZeroCounts: int
    driver_mA: int
    maxRPS: float
    Kp: float
    Ki: float
    Kd: float
    flags: int
    canArbId: int

    @staticmethod
    def from_bytes(b: bytes) -> "AxisConfigWire":
        vals = struct.unpack(AXIS_FMT, b[:AXIS_SIZE])
        return AxisConfigWire(*vals)

# ---- DataPacket = 32B config + 3 doubles (24B) = 56B ----
DP_FMT = "<d d d"   # 3 doubles
DP_SIZE = struct.calcsize(DP_FMT)

@dataclass
class DataPacket:
    config: AxisConfigWire
    currentSpeed: float
    currentAngle: float
    targetAngle: float

    @staticmethod
    def from_payload(pay: bytes) -> Optional["DataPacket"]:
        if len(pay) < AXIS_SIZE + DP_SIZE:
            return None
        cfg = AxisConfigWire.from_bytes(pay[0:AXIS_SIZE])
        speed, angle, target = struct.unpack(DP_FMT, pay[AXIS_SIZE:AXIS_SIZE+DP_SIZE])
        return DataPacket(cfg, speed, angle, target)

# ---- Serial framing helpers ----
def read_exact(ser: serial.Serial, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)

def read_one_frame(ser: serial.Serial):
    hdr = read_exact(ser, HDR_SIZE)
    if hdr is None:
        return None
    id_l, id_h, cmd, ln = hdr
    pay = read_exact(ser, ln)
    if pay is None:
        return None
    can_id = id_l | (id_h << 8)
    return can_id, cmd, ln, pay

# ---- Main loop ----
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--show-raw", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print("Listening… Ctrl-C to stop")
    try:
        while True:
            frm = read_one_frame(ser)
            if frm is None: break
            can_id, cmd, ln, pay = frm

            if args.show_raw:
                print(f"RAW ID=0x{can_id:03X} CMD=0x{cmd:02X} LEN={ln} PAY={pay.hex(' ')}")

            if cmd == CMD_TELEM:
                dp = DataPacket.from_payload(pay)
                if not dp:
                    print("Frame too short")
                    continue
                print(f"[ID=0x{can_id:03X}] speed={dp.currentSpeed:.3f}, "
                      f"angle={dp.currentAngle:.3f}, target={dp.targetAngle:.3f}")
                print(f"  cfg: µsteps={dp.config.microsteps}, units={dp.config.units}, "
                      f"driver_mA={dp.config.driver_mA}, maxRPS={dp.config.maxRPS:.3f}, "
                      f"Kp={dp.config.Kp:.3f}, Ki={dp.config.Ki:.3f}, Kd={dp.config.Kd:.3f}, "
                      f"canArbId=0x{dp.config.canArbId:03X}, crc32=0x{dp.config.crc32:08X}")
            else:
                print(f"[ID=0x{can_id:03X}] CMD=0x{cmd:02X} LEN={ln}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()