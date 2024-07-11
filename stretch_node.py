#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String

import util
from util import Intents

# look at stretch driver node --> i think need to pub to that

class AlexaCommands(Node):
    #### Constants

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

    def __init__(self):
        super().__init__('stretch_alexa_commands_node')
        self.publisher_ = self.create_publisher(String, 'alexa_cmd', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.move_around)

    def issue_alexa_command(self, intent):
        if intent == Intents.STOP:
            self.stop()
        if intent == Intents.STOW:
            self.stow()
        if intent == Intents.SCAN_ROOM:
            self.scan_room()
        if intent == Intents.REACH_TABLE:
            self.reach_table
        if intent == Intents.MOVE_TO_TABLE:
            self.get_table()
        if intent == Intents.GRAB_FROM_GROUND:
            self.get_object()

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

    def get_table(self):
        # something w/ cameras
        angle = 1
        dist = 1
        self.move_to_table(angle, dist)

    def move_to_table(self, angle, dist):

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

        self.reach_table()

    def get_object(self):
        # something w/ cameras
        angle = 1
        dist = 1
        self.grab_from_ground(angle, dist)
        
    def grab_from_ground(self, angle, dist):
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
        r.lift.move_to(self.TABLE_HEIGHT)
        r.base.push_command()

def main(args=[]):
    rclpy.init(args=args)
    alexa_commands = AlexaCommands()
    rclpy.spin(alexa_commands)
    alexa_commands.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()