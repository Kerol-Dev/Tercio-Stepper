from TercioStepperLib import Stepper, Bridge, HomingParams
import time

# ---------- Setup ----------
bridge = Bridge(port="COM4")      # adjust port
bridge.open()

homingparams = HomingParams()
homingparams.activeLow = True
homingparams.direction = False
homingparams.useMINTrigger = True
homingparams.offset = 3.14
homingparams.speed = 0.5 

stepper = Stepper(bridge=bridge, can_id=0x001)
stepper.enable_motor(True)
stepper.set_direction_invert(True)
stepper.set_current_ma(1500)
stepper.set_microsteps(256)
stepper.set_accel_limit_rps2(250)
stepper.set_speed_limit_rps(10)
stepper.set_target_angle(10)

while True:
    if(stepper.get_axis_state() is not None):
        print(stepper.get_axis_state().config.flags.dirInvert)