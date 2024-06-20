import stretch_body.robot
import time
import math

# setup and start stretch
r = stretch_body.robot.Robot()
r.startup()

# lift height constants
TABLE_HEIGHT = 1.1 # avg table height + length of gripper (also max height by chance)
LIFT_MID_HEIGHT = .55  
GROUND_REACH_HEIGHT = 0.2 

# velocity constants
BASE_TRANS_VEL = .05
BASE_ROT_VEL = 0.1
BASE_STOP_VEL = 0

# stretch robot dimensions
BASE_LENGTH = .34
BASE_WIDTH = .33

# table lengths
ARM_OUT_TO_TABLE_LEN = .3

if not r.startup():
    exit() # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()

def stop():
    r.stop()

def stow():
    r.stow()

def scan_room():
    r.head.move_to('head_pan', 1.77)
    r.push_command()
    time.sleep(.5)
    r.head.move_by('head_pan', -4.1, 0.2)
    r.push_command()

def reach_table():

    # set up arm and lift
    # r.head.move_to('head_pan', math.pi / -2)
    r.lift.move_to(TABLE_HEIGHT)
    r.arm.move_to(ARM_OUT_TO_TABLE_LEN)
    r.lift.motor.wait_until_at_setpoint()
    r.arm.motor.wait_until_at_setpoint()
    r.push_command()

    # tilt wrist and open gripper
    r.end_of_arm.move_to('wrist_pitch', -1.5) # point down
    r.end_of_arm.move_to('stretch_gripper', 50) # i think has range of -50 (closed) to 50 (open)
    r.push_command()

def move_to_table(angle, dist):

    move_time = BASE_TRANS_VEL / dist
    dist = dist - ARM_OUT_TO_TABLE_LEN

    r.stow()
    r.push_command()

    # move to table
    r.base.rotate_by(angle, BASE_TRANS_VEL)
    r.base.left_wheel.wait_until_at_setpoint()
    r.base.translate_by(dist, BASE_ROT_VEL)
    r.base.push_command()
    time.sleep(move_time)

    # set arm and gripper
    r.arm.move_to(0)
    r.lift.move_to(TABLE_HEIGHT)
    r.end_of_arm.move_to('stretch_gripper', 0)
    r.end_of_arm.move_to('wrist_pitch', 0)
    r.push_command()

    reach_table()
    
def grab_from_ground(angle, dist):
    ARM_OUT_LEN = .2

    dist = dist - ARM_OUT_LEN
    move_time = BASE_TRANS_VEL / dist

    return_angle = math.pi - angle
    return_dist = dist * -1

    r.stow()
    r.push_command()

    # move to object
    r.base.rotate_by(angle, BASE_ROT_VEL)
    r.base.left_wheel.wait_until_at_setpoint()
    r.base.translate_by(dist, BASE_TRANS_VEL)
    r.base.push_command()
    time.sleep(move_time)

    # set up for grab
    r.head.pose('wheels')
    r.arm.move_to(ARM_OUT_LEN)
    r.end_of_arm.move_to('wrist_pitch', -1.5)
    r.end_of_arm.move_to('stretch_gripper', 50) # how to open it wider??
    r.push_command()
    time.sleep(1)
    r.lift.move_to(GROUND_REACH_HEIGHT)
    r.lift.motor.wait_until_at_setpoint()
    r.push_command()

    # pick up object
    r.end_of_arm.move_to('stretch_gripper', -50)
    time.sleep(.5)
    r.lift.move_to(LIFT_MID_HEIGHT)
    r.base.push_command()
    time.sleep(1)

    r.stow()
    r.push_command()

    # go back to start pos
    r.base.rotate_by(return_angle)
    r.base.translate_by(return_dist)
    r.lift.move_to(TABLE_HEIGHT)
    r.base.push_command()