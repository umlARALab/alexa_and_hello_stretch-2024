import stretch_body.robot
import time
import math

# setup and start stretch
r = stretch_body.robot.Robot()
r.startup()

# lift height constants
TABLE_HEIGHT = .75
LIFT_MID_HEIGHT = .55  
GROUND_REACH_HEIGHT = 0  

# velocity constants
BASE_TRANS_VEL = .05
BASE_ROT_VEL = 0.1
BASE_STOP_VEL = 0

# stretch robot dimensions
BASE_LENGTH = .34
BASE_WIDTH = .33

if not r.startup():
    exit() # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()

def stop():
    r.stop()

def scan_room():

    # set up lift and arm posistion and vel
    r.lift.move_to(LIFT_MID_HEIGHT)
    r.base.set_rotational_velocity(BASE_ROT_VEL)
    r.head.pose('ahead')
    r.push_command()
    r.lift.motor.wait_until_at_setpoint()

    # spin around
    r.base.rotate_by(2 * math.pi)
    r.base.push_command()
    r.base.left_wheel.wait_until_at_setpoint()

    stop()

def reach_table():
    ARM_OUT_LEN = .6

    # set up arm and lift
    r.lift.move_to(TABLE_HEIGHT)
    r.head.pose('ahead')
    r.arm.move_to(ARM_OUT_LEN)
    r.push_command()
    r.lift.motor.wait_until_at_setpoint()
    r.arm.motor.wait_until_at_setpoint()

    # tilt wrist and open gripper
    r.head.pose('wheels')
    r.end_of_arm.move_to('wrist_pitch', -0.5)
    r.end_of_arm.move_to('stretch_gripper', 50)
    r.push_command()

def move_to_table(angle, dist):

    move_time = BASE_TRANS_VEL / dist
    dist = dist - ARM_OUT_LEN - BASE_LENGTH

    # set vel
    r.base.set_rotational_velocity(BASE_ROT_VEL)
    r.base.set_translate_velocity(BASE_TRANS_VEL)
    r.push_command()
    time.sleep(1)

    # move to table
    r.base.rotate_by(angle)
    r.base.left_wheel.wait_until_at_setpoint()
    r.base.translate_by(dist)
    r.base.push_command()
    time.sleep(move_time)

    stop()
    
def grab_from_ground(angle, dist):
    ARM_OUT_LEN = .30

    dist = dist - ARM_OUT_LEN - BASE_LENGTH
    move_time = BASE_TRANS_VEL / dist

    return_angle = math.pi - angle
    return_dist = dist * -1

    # set vel and start posistions
    r.base.set_rotational_velocity(BASE_ROT_VEL)
    r.base.set_translate_velocity(BASE_TRANS_VEL)
    r.lift.move_to(GROUND_REACH_HEIGHT)
    r.head.pose('wheel')
    r.arm.move_to(ARM_OUT_LEN)
    r.end_of_arm.move_to('wrist_pitch', -0.5)
    r.end_of_arm.move_to('stretch_gripper', 50)
    r.push_command()
    time.sleep(1)

    # move to object
    r.base.rotate_by(angle)
    r.base.left_wheel.wait_until_at_setpoint()
    r.base.translate_by(dist)
    r.base.push_command()
    time.sleep(move_time)

    # pick up object
    r.end_of_arm.move_to('stretch_gripper', -50)
    time.sleep(.5)
    r.lift.move_to(LIFT_MID_HEIGHT)
    r.base.push_command()
    time.sleep(1)

    # go back to start pos
    r.base.rotate_by(return_angle)
    r.base.translate_by(return_dist)
    r.lift.move_to(TABLE_HEIGHT)
    r.base.push_command()

    stop()