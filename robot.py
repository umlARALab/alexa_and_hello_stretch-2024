import stretch_body.robot
import time

r = stretch_body.robot.Robot()
r.startup()

if not r.startup():
    exit() # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()

def stop():
    r.stop()

def scan_room():
    LIFT_MID_HEIGHT = .55
    BASE_ROT_VEL = 0.1
    BASE_STOP_VEL = 0

    r.lift.move_to(LIFT_MID_HEIGHT)
    r.base.set_rotational_velocity(BASE_ROT_VEL)
    r.head.pose('ahead')
    r.push_command()
    r.lift.motor.wait_until_at_setpoint()
    r.base.rotate_by(3.14)
    r.base.push_command()
    r.base.left_wheel.wait_until_at_setpoint()
    r.base.set_rotational_velocity(BASE_STOP_VEL)


def reach_table():
    TABLE_HEIGHT = .75
    ARM_OUT_LEN = .6

    r.lift.move_to(TABLE_HEIGHT)
    r.head.pose('ahead')
    r.arm.move_to(ARM_OUT_LEN)
    r.push_command()
    r.lift.motor.wait_until_at_setpoint()
    r.arm.motor.wait_until_at_setpoint()
    r.head.pose('wheels')
    r.end_of_arm.move_to('wrist_pitch', -0.5)
    r.end_of_arm.move_to('stretch_gripper', 50)

def move_to_table(angle, dist):
    BASE_TRANS_VEL = .05
    move_time = BASE_TRANS_VEL / dist

    r.base.set_rotational_velocity(0.1)
    r.push_command()
    time.sleep(1)
    r.base.rotate_by(angle)
    r.base.left_wheel.wait_until_at_setpoint()
    r.base.set_translate_velocity(0.05)
    time.sleep(move_time)
    # stop base motion
    r.base.enable_freewheel_mode()
    r.base.push_command()
    
