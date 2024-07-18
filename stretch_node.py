import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_srvs.srv import Trigger

import alexa_and_stretch.util as util
from alexa_and_stretch.util import Intents
import time
import math

import stretch_body
import stretch_body.robot as rb

#### Constants
tables = []
objects = []
chosen_obj = -1
chosen_table = -1

class AlexaCommands(Node):

    def __init__(self):
        super().__init__("stretch_alexa_commands_node")

        self.publisher = self.create_publisher(Int32, "num_tables_topic", 10)
        self.publisher = self.create_publisher(Int32, "num_objects_topic", 10)

        self.subscribtion = self.create_subscription(
            Int32, "selected_intent_topic", self.intent_listener_callback, 10
        )
        self.subscribtion = self.create_subscription(
            Int32, "selected_table_topic", self.table_listener_callback, 10
        )
        self.subscribtion = self.create_subscription(
            Int32, "selected_object_topic", self.object_listener_callback, 10
        )
        self.subscribtion

        self.get_logger().info("Initialized")
        self.create_timer(1.0, self.publish_message)

        self.robot = rb.Robot()
        self.robot.startup()

    def intent_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        num = int(f"{msg.data}")
        self.issue_alexa_command(num)

    def table_listener_callback(self, msg):
        global chosen_table
        self.get_logger().info('I heard: "%s"' % msg.data)
        num = int(f"{msg.data}")
        chosen_table = num

    def object_listener_callback(self, msg):
        global chosen_object
        self.get_logger().info('I heard: "%s"' % msg.data)
        num = int(f"{msg.data}")
        chosen_object = num

    def issue_alexa_command(self, intent):

        print(intent)
        self.publish_message
        if intent == Intents.STOP.value:
            self.get_logger().info("stop")
            # self.robot.stop()
        elif intent == Intents.STOW.value:
            self.get_logger().info("stow")
            # self.robot.stow()
        elif intent == Intents.SCAN_ROOM.value:
            self.get_logger().info("scan")
            # self.scan_room()
        elif intent == Intents.REACH_TABLE.value:
            self.get_logger().info("reach")
            # self.reach_table
        elif intent == Intents.MOVE_TO_TABLE.value:
            self.get_logger().info("move table")
            self.get_table()
        elif intent == Intents.GRAB_FROM_GROUND.value:
            self.get_logger().info("grab obj")
            self.get_object()
        elif intent == Intents.GET_TABLES.value:
            self.get_logger().info("get table")
            self.look_for_tables()
        elif intent == Intents.GET_OBJECTS.value:
            self.get_logger().info("get obj")
            self.look_for_objects()
        elif intent == Intents.TEST_LIFT_SMALL.value:
            self.get_logger().info("small")
            # self.move_lift_small()

        # self.robot.stop()

    def move_lift_small(self):
        self.robot.lift.move_to(0.5)
        self.robot.push_command()

    def scan_room(self):
        self.robot.head.move_to("head_pan", 1.77)
        self.robot.push_command()
        time.sleep(0.5)
        self.robot.head.move_by("head_pan", -4.1, 0.2)
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
        self.robot.end_of_arm.move_to("wrist_pitch", -1.5)  # point down
        self.robot.end_of_arm.move_to(
            "stretch_gripper", 50
        )  # i think it has a range of -50 (closed) to 50 (open)
        self.robot.push_command()

    def look_for_tables(self):
        global tables
        # something w/ cameras
        tables = [(0,0), (1,1), (2,2), (3,3)]

    def get_table(self):
        global chosen_table
        table = objects[chosen_table]

        angle = table[0]
        dist = table[1]
        print(table)
        # self.move_to_table(angle, dist)

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
        self.robot.end_of_arm.move_to("stretch_gripper", 0)
        self.robot.end_of_arm.move_to("wrist_pitch", 0)
        self.robot.push_command()

        self.reach_table()

    def look_for_objects(self):
        global objects
        # something w/ cameras
        objects = [(0,0), (1,1), (2,2), (3,3)]

    def get_object(self):
        global chosen_obj
        object = objects[chosen_obj]

        angle = object[0]
        dist = object[1]
        print(object)
        # self.grab_from_ground(angle, dist)

    def grab_from_ground(self, angle, dist):
        ARM_OUT_LEN = 0.2

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
        self.robot.head.pose("wheels")
        self.robot.arm.move_to(util.ARM_OUT_LEN)
        self.robot.end_of_arm.move_to("wrist_pitch", -1.5)
        self.robot.end_of_arm.move_to("stretch_gripper", 50)  # how to open it wider??
        self.robot.push_command()
        time.sleep(1)
        self.robot.lift.move_to(util.GROUND_REACH_HEIGHT)
        self.robot.lift.motor.wait_until_at_setpoint()
        self.robot.push_command()

        # pick up object
        self.robot.end_of_arm.move_to("stretch_gripper", -50)
        time.sleep(0.5)
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
    
    def publish_message(self):
        self.publish_objects_message()
        self.publish_tables_message()

    def publish_objects_message(self):
        global objects

        num_objects = len(objects)

        msg = Int32()
        if num_objects >= 0:
            msg.data = num_objects
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing num_objects: {msg.data}")
            num_objects = -1

    def publish_tables_message(self):
        global tables

        num_tables = len(tables)

        msg = Int32()
        if num_tables >= 0:
            msg.data = num_tables
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing num_tables: {msg.data}")
            num_tables = -1

def main(args=[]):
    rclpy.init(args=args)
    alexa_commands = AlexaCommands()
    rclpy.spin(alexa_commands)
    alexa_commands.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
