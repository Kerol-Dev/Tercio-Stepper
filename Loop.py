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
stepper.set_current_ma(1200)
stepper.set_microsteps(256)
stepper.set_accel_limit_rps2(250)
stepper.set_speed_limit_rps(10)
stepper.set_pid(3, 0, 0)
stepper.set_units_degrees(False)
stepper.set_endstop(True)
stepper.set_external_encoder(False)
stepper.set_target_angle(-1000)


while True:
    if(stepper.get_axis_state() is not None):
        print(stepper.get_axis_state().currentAngle)
        time.sleep(0.2)
