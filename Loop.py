from TercioStepperLib import Stepper
from TercioStepperLib import Bridge

# Define the stepper motor and bridge
bridge = Bridge(verbose=True)
stepper = Stepper(bridge=bridge, motor_id=0x001)

# Initialize the stepper motor
stepper.set_current_ma(1000)  # Set motor current to 800mA
stepper.set_microsteps(256)   # Set microstepping to 1/256
stepper.set_speed_limit_rps(5)  # Set speed limit to 5 revolutions per second
stepper.set_units_degrees(True)  # Use degrees for movement commands
stepper.do_calibrate()
stepper.set_target_angle(1000, True)  # Move to 1000 degrees and wait for completion

while True:
    axistate = stepper.get_axis_state()
    print(f"Position: {axistate.currentAngle}, Velocity: {axistate.currentSpeed}")
    print(f"Target: {axistate.targetAngle}, ID: {axistate.config.canArbId}")