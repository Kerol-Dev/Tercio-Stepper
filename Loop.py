import time
import argparse
from TercioStepperLib import Bridge

def parse_args():
    p = argparse.ArgumentParser(description="Basic setup, calibrate, move, and stream telemetry.")
    p.add_argument("--port", default="COM4", help="Serial port (e.g. COM4 or /dev/ttyACM0)")
    p.add_argument("--id", type=lambda x: int(x, 0), default=0x001, help="Device CAN ID (e.g. 0x001)")
    p.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    p.add_argument("--angle", type=float, default=45.0, help="Target angle (device interprets per cfg.units)")
    p.add_argument("--current_ma", type=int, default=800, help="Motor RMS current (mA)")
    p.add_argument("--microsteps", type=int, default=256, help="Microsteps (1..256)")
    p.add_argument("--speed_limit", type=float, default=5.0, help="Max speed limit (rps)")
    p.add_argument("--degrees", action="store_true", default=True, help="Use degrees (device-side units)")
    return p.parse_args()

def main():
    args = parse_args()
    br = Bridge(args.port, baudrate=args.baud, verbose=False, auto_reader=True)
    br.open()
    try:
        # --- Basic setup ---
        br.enable_motor(args.id, True)
        br.set_units_degrees(args.id, args.degrees)
        br.set_current_ma(args.id, args.current_ma)
        br.set_microsteps(args.id, args.microsteps)
        br.set_speed_limit_rps(args.id, args.speed_limit)

        # Optional: set full steps if your device expects it (uncomment and set your motor steps)
        # br.set_steps_per_rev(args.id, 200)

        # --- Calibration ---
        br.do_calibrate(args.id)
        time.sleep(0.5)  # brief settle; adjust to your firmware’s routine

        # --- Move to target ---
        br.set_target_angle(args.id, args.angle, degrees=args.degrees)

        print(f"Streaming telemetry for CAN ID 0x{args.id:03X} (Ctrl+C to stop)")
        # Warm-up: wait until we see at least one packet for this device
        _ = br.get_axis_state(args.id, timeout_s=2.0)

        # --- Telemetry loop ---
        while True:
            st = br.get_axis_state(args.id, timeout_s=0.5)
            if st is None:
                print("No telemetry yet...")
            else:
                ang = st.currentAngle if st.currentAngle is not None else float("nan")
                tgt = st.targetAngle  if st.targetAngle  is not None else float("nan")
                spd = st.currentSpeed if st.currentSpeed is not None else float("nan")
                print(f"ID 0x{args.id:03X} | angle={ang:9.4f} | target={tgt:9.4f} | speed={spd:8.5f} rps")
                print(st.temperature)

    except KeyboardInterrupt:
        pass
    finally:
        br.close()

if __name__ == "__main__":
    main()