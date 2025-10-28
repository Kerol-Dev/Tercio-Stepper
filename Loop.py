from TercioStepperLib import Bridge, Stepper
import time

# --- Setup bridge ---
bridge = Bridge(port="COM4", baud=115200)
bridge.open()

# --- Two steppers (IDs 0x1 and 0x2) ---
m1 = Stepper(bridge, 0x001)
m2 = Stepper(bridge, 0x002)

# --- Enable both and set basic limits ---
for m in (m1, m2):
    m.enable_motor(True)
    m.set_units_degrees(True)
    m.set_current_ma(3000)
    m.set_microsteps(256)
    m.set_speed_limit_rps(20.0)
    m.set_accel_limit_rps2(100.0)
    m.set_pid(5.0, 0.0, 0.0)

# --- Simple alternating move ---
ANGLE = 900.0
DELAY = 0.01  # seconds to let telemetry update
PAUSE = 0.05  # rest between moves

def wait_until_reached(motor: Stepper, target_deg: float, tol: float = 3.0, timeout: float = 5.0):
    """Wait until currentAngle is within tolerance of targetAngle."""
    start = time.time()
    while time.time() - start < timeout:
        st = motor.get_axis_state()
        if st:
            err = abs(st.currentAngle - target_deg)
            if err <= tol:
                return True
        time.sleep(0.05)
    return False

try:
    direction = 1
    while True:
        # --- first motor moves ---
        target1 = ANGLE * direction
        print(f"\n[M1] → {target1:.1f}°")
        m1.set_target_angle(target1)
        wait_until_reached(m1, target1)
        time.sleep(PAUSE)

        # --- second motor moves ---
        target2 = ANGLE * direction
        print(f"[M2] → {target2:.1f}°")
        m2.set_target_angle(target2)
        wait_until_reached(m2, target2)
        time.sleep(PAUSE)

        # --- reverse direction ---
        direction *= -1

except KeyboardInterrupt:
    print("\nStopping...")
    m1.enable_motor(False)
    m2.enable_motor(False)
    bridge.close()