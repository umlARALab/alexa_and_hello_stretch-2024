import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import util
from util import Intents
import time
import math

import stretch_body
import stretch_body.robot

## have alexa commands node just recieve the intent name and then leave the rest for now, make more groupings

class AlexaCommands(Node):
    #### Constants

    def __init__(self):
        super().__init__('stretch_alexa_commands_node')

        #start_publish?

        # self.robot = StretchBody()
        # self.robot.start_publish()
        

        self.subscribtion = self.create_subscription(
            String,
            'selected_intent_topic',
            self.listener_callback,
            10)
        self.subscribtion

        self.robot = stretch_body.robot.Robot()
        self.robot.start()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.issue_alexa_command(msg.data)


    def issue_alexa_command(self, intent):
        # self.publish_message
        if intent == Intents.STOP:
            self.robot.stop()
        if intent == Intents.STOW:
            self.robot.stow()
        if intent == Intents.SCAN_ROOM:
            self.scan_room()
        if intent == Intents.REACH_TABLE:
            self.reach_table
        if intent == Intents.MOVE_TO_TABLE:
            self.get_table()
        if intent == Intents.GRAB_FROM_GROUND:
            self.get_object()

    def scan_room(self):
        self.robot.head.move_to('head_pan', 1.77)
        self.robot.push_command()
        time.sleep(.5)
        self.robot.head.move_by('head_pan', -4.1, 0.2)
        self.robot.push_command()

    def reach_table(self):

        # set up arm and lift
        # r.head.move_to('head_pan', math.pi / -2)
        self.robot.lift.move_to(util.TABLE_HEIGHT)
        self.robot.arm.move_to(util.ARM_OUT_TO_TABLE_LEN)
        self.robot.lift.motor.wait_until_at_setpoint()
        self.robot.arm.motor.wait_until_at_setpoint()
        self.robot.push_command()

        # tilt wrist and open gripper
        self.robot.end_of_arm.move_to('wrist_pitch', -1.5) # point down
        self.robot.end_of_arm.move_to('stretch_gripper', 50) # i think has range of -50 (closed) to 50 (open)
        self.robot.push_command()

    def get_table(self):
        # something w/ cameras
        angle = 1
        dist = 1
        self.move_to_table(angle, dist)

    def move_to_table(self, angle, dist):

        move_time = util.BASE_TRANS_VEL / dist
        dist = dist - util.ARM_OUT_TO_TABLE_LEN

        self.robot.stow()
        self.robot.push_command()

        # move to table
        self.robot.base.rotate_by(angle, util.BASE_TRANS_VEL)
        self.robot.base.left_wheel.wait_until_at_setpoint()
        self.robot.base.translate_by(dist, util.BASE_ROT_VEL)
        self.robot.base.push_command()
        time.sleep(move_time)

        # set arm and gripper
        self.robot.arm.move_to(0)
        self.robot.lift.move_to(util.TABLE_HEIGHT)
        self.robot.end_of_arm.move_to('stretch_gripper', 0)
        self.robot.end_of_arm.move_to('wrist_pitch', 0)
        self.robot.push_command()

        self.reach_table()

    def get_object(self):
        # something w/ cameras
        angle = 1
        dist = 1
        self.grab_from_ground(angle, dist)
        
    def grab_from_ground(self, angle, dist):
        ARM_OUT_LEN = .2

        dist = dist - ARM_OUT_LEN
        move_time = util.BASE_TRANS_VEL / dist

        return_angle = math.pi - angle
        return_dist = dist * -1

        self.robot.stow()
        self.robot.push_command()

        # move to object
        self.robot.base.rotate_by(angle, util.BASE_ROT_VEL)
        self.robot.base.left_wheel.wait_until_at_setpoint()
        self.robot.base.translate_by(dist, util.BASE_TRANS_VEL)
        self.robot.base.push_command()
        time.sleep(move_time)

        # set up for grab
        self.robot.head.pose('wheels')
        self.robot.arm.move_to(util.ARM_OUT_LEN)
        self.robot.end_of_arm.move_to('wrist_pitch', -1.5)
        self.robot.end_of_arm.move_to('stretch_gripper', 50) # how to open it wider??
        self.robot.push_command()
        time.sleep(1)
        self.robot.lift.move_to(util.GROUND_REACH_HEIGHT)
        self.robot.lift.motor.wait_until_at_setpoint()
        self.robot.push_command()

        # pick up object
        self.robot.end_of_arm.move_to('stretch_gripper', -50)
        time.sleep(.5)
        self.robot.lift.move_to(util.LIFT_MID_HEIGHT)
        self.robot.base.push_command()
        time.sleep(1)

        self.robot.stow()
        self.robot.push_command()

        # go back to start pos
        self.robot.base.rotate_by(return_angle)
        self.robot.base.translate_by(return_dist)
        self.robot.lift.move_to(util.TABLE_HEIGHT)
        self.robot.base.push_command()

def main(args=[]):
    rclpy.init(args=args)
    alexa_commands = AlexaCommands()
    rclpy.spin(alexa_commands)
    alexa_commands.destroy_node()
    rclpy.shutdown()
    self.robot.stop()

if __name__ == '__main__':
    main()