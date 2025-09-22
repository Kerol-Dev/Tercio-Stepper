from TercioLibStepper import StepperLink

dev = StepperLink("COM4")

# send commands
dev.set_units(1)
dev.set_microsteps(16)
dev.set_current_ma(1200)
dev.set_steps_per_rev(200)
dev.set_pid(3.0, 0.0, 0.0)
dev.set_speed_limit_rps(4.0)
dev.set_enabled(True)
dev.set_target_angle(90.0)

# read & decode telemetry frames (from sendData)
msg = dev.read(timeout=0.2)  # (cmd, payload)
if msg >= 58:
    telem = StepperLink.parse_datapacket(msg[1])
    cfg = telem["config"]
    # cfg['microsteps'], cfg['stepsPerRev'], cfg['units'], cfg['encInvert'], ...
    # telem['currentSpeed'], telem['currentAngle'], telem['targetAngle']
    for key, value in cfg.items():
        print(f"{key}: {value}")
dev.close()
