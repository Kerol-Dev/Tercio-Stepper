from TercioStepperLib import Stepper, Bridge
import time

# ---------- Setup ----------
bridge = Bridge(verbose=True, port="COM4")   # adjust COM port
bridge.open()

stepper = Stepper(bridge=bridge, can_id=0x001)
stepper.set_current_ma(1000)
stepper.set_microsteps(256)
stepper.set_accel_limit_rps2(250)
stepper.set_speed_limit_rps(35)
stepper.set_pid(5, 0, 0)
stepper.set_units_degrees(True)
stepper.do_calibrate()

# ---------- Motion parameters ----------
target_angle_forward = 9000.0
target_angle_backward = -9000.0
tolerance_deg = 1.0

def wait_until_reached(target, tol=tolerance_deg):
    """Waits until the motor reaches the target within tolerance."""
    while True:
        state = stepper.get_axis_state()
        diff = abs(state.currentAngle - target)
        print(f"Target: {target:.1f}°, Current: {state.currentAngle:.2f}°, Vel: {state.currentSpeed:.2f}°/s")
        if diff <= tol and abs(state.currentSpeed) < 0.1:
            break
        time.sleep(0.05)  # 20Hz refresh

# ---------- Main loop ----------
while True:
    print("\n➡ Moving +180°...")
    stepper.set_target_angle(target_angle_forward)
    wait_until_reached(target_angle_forward)

    print("\n⬅ Moving back to 0°...")
    stepper.set_target_angle(target_angle_backward)
    wait_until_reached(target_angle_backward)
