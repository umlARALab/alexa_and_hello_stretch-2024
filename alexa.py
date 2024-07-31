#!/usr/bin/env python3

from flask import Flask, render_template, jsonify, request

# imports needed for ASK SDK
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import (
    AbstractRequestHandler,
    AbstractExceptionHandler,
)
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.dialog import ElicitSlotDirective, DelegateDirective
from ask_sdk_model import (
    Response,
    IntentRequest,
    DialogState,
    SlotConfirmationStatus,
    Slot,
    ui,
    IntentConfirmationStatus,
)
from ask_sdk_model.ui import SimpleCard
from ask_sdk_model.intent import Intent

# imports needed for ROS2 node
import rclpy
from rclpy.node import Node
import threading
import rclpy.publisher
from std_msgs.msg import Int32, String, Float32, Bool

from alexa_and_stretch.util import Intents
# from util import Intents

# port for local hosting
PORT = 9999

### run these commands in two terminals to start hosting
# ngrok http 9999 to start
# python3 alexa.py

app = Flask(
    __name__,
    # location of html template on Stretch
    template_folder="/home/csrobot/ament_ws/src/alexa_and_stretch/alexa_and_stretch/templates/",
)

node_data = {"intent": 1111}

sb = SkillBuilder()

# VERIFY_TIMESTAMP_APP_CONFIG = False

# fillers for html page
current_intent = ""
current_movement = ""

# custom intent variables
custom_intent = None
custom_intent_array = [["Custom Action 1"], ["Custom Action 2"], ["Custom Action 3"]]
send_custom_intent = False
custom_intent_num = -1

# variables for user selection
num_objects = 0
chosen_object = -1
demo_cube_height = -1
close_gripper = False

num_tables = 0
chosen_table = -1

@app.route("/")
def homepage():
    global current_intent, custom_intent, current_movement, node_data

    custom_text = "Choose a Custom Action to build!"

    if custom_intent is not None:
        custom_text = ", ".join(custom_intent)

    return render_template(
        "index.html",
        intent=current_intent,
        movements=current_movement,
        custom=custom_text,
    )


# --------------------------------------------------------------------
# Amazon Alexa Handlers
# --------------------------------------------------------------------

@sb.request_handler(can_handle_func=is_request_type("LaunchRequest"))
def launch_request_handler(handler_input):
    global current_intent
    current_intent = "Launch Request"

    speech_text = "Welcome to the Alexa Skills Kit, you can say hello!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)
    ).set_should_end_session(False)
    return handler_input.response_builder.response


@sb.request_handler(can_handle_func=is_intent_name("HelloWorldIntent"))
def hello_world_intent_handler(handler_input):
    global current_intent
    current_intent = "Hello World Intent"

    speech_text = "Stretch says hello!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)
    ).set_should_end_session(True)

    return handler_input.response_builder.response


@sb.request_handler(can_handle_func=is_intent_name("AMAZON.HelpIntent"))
def help_intent_handler(handler_input):
    global current_intent
    current_intent = "Amazon Help Intent"

    speech_text = "You can say hello to me!"

    handler_input.response_builder.speak(speech_text).ask(speech_text).set_card(
        SimpleCard("Hello World", speech_text)
    )
    return handler_input.response_builder.response


@sb.request_handler(
    can_handle_func=lambda handler_input: is_intent_name("AMAZON.CancelIntent")(
        handler_input
    )
    or is_intent_name("AMAZON.StopIntent")(handler_input)
)
def cancel_and_stop_intent_handler(handler_input):
    global current_intent
    current_intent = "Amazon Cancel or Stop Intent"

    speech_text = "Goodbye!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)
    ).set_should_end_session(True)
    return handler_input.response_builder.response


@sb.request_handler(can_handle_func=is_request_type("SessionEndedRequest"))
def session_ended_request_handler(handler_input):
    global current_intent
    current_intent = "Session End Request"

    return handler_input.response_builder.response


@sb.exception_handler(can_handle_func=lambda i, e: True)
def all_exception_handler(handler_input, exception):
    global current_intent
    current_intent = "Exception Handler"

    print(exception)

    speech = "Sorry, I didn't get it. Can you please say it again!!"
    handler_input.response_builder.speak(speech).ask(speech)
    return handler_input.response_builder.response


# --------------------------------------------------------------------
# Hello Stretch Alexa Intent Handlers
# --------------------------------------------------------------------

@sb.request_handler(can_handle_func=is_intent_name("ScanRoom"))
def scan_room_intent_handler(handler_input):

    # set_intent_info(Intents.SCAN_ROOM)
    set_intent_info(Intents.TEST_LIFT_SMALL)

    speech_text = "Scanning the room!"

    handler_input.response_builder.speak(speech_text)
    return handler_input.response_builder.response


@sb.request_handler(can_handle_func=is_intent_name("GrabFromTable"))
def grab_from_table_intent_handler(handler_input):

    set_intent_info(Intents.GRAB_FROM_GROUND)

    print(current_movement)

    speech_text = "Grab from table!"

    handler_input.response_builder.speak(speech_text)
    return handler_input.response_builder.response


@sb.request_handler(can_handle_func=is_intent_name("MoveToTable"))
def move_to_table_intent_handler(handler_input):
    global num_tables

    set_intent_info(Intents.GET_TABLES)

    table_loc = handler_input.request_envelope.request.intent.slots["table_loc"].value

    if num_tables == 1:
        # table = choose_table(0)
        # move_to_table(table[0], table[1])
        speech_text = "I only see one table. Moving to the table."
        return handler_input.response_builder.speak(speech_text).response
    elif table_loc == None:
        if num_tables == 2:
            speech_text = (
                "I found "
                + str(num_tables)
                + " tables. Would you like the closest or farthest?"
            )
            reprompt = "Would you like the closest or farthest?"
            # handler_input.response_builder.speak(speech_text).add_directive(DelegateDirective(updated_intent=Intent(name="ChooseTable")))
            return (
                handler_input.response_builder.speak(speech_text)
                .ask(reprompt)
                .add_directive(
                    ElicitSlotDirective(
                        updated_intent=Intent(name="ChooseTable"),
                        slot_to_elicit="table_loc",
                    )
                )
                .response
            )
        else:
            speech_text = (
                "I found "
                + str(num_tables)
                + " tables. Would you like the closest, farthest, or another one?"
            )
            reprompt = "Would you like the closest, farthest, or another one?"
            handler_input.response_builder.ask(reprompt).add_directive(
                ElicitSlotDirective(
                    updated_intent=Intent(name="ChooseTable"),
                    slot_to_elicit="table_loc",
                )
            )
    else:
        if table_loc == "farthest":
            speech_text = "Ok. Moving to the farthest table."
            # table = choose_table(0)
        elif table_loc == "closest":
            speech_text = "Ok. Moving to the closest table."
            # table = choose_table(num_tables)

    return handler_input.response_builder.speak(speech_text).response


@sb.request_handler(can_handle_func=is_intent_name("ChooseTable"))
def choose_table_intent_handler(handler_input):
    global chosen_table

    table_loc = (
        handler_input.request_envelope.request.intent.slots["table_loc"]
        .resolutions.resolutions_per_authority[0]
        .values[0]
        .value.name
    )

    speech_text = "There are tables."

    if table_loc == "farthest":
        speech_text = "Ok. Moving to the farthest table."
        chosen_table = 0
    elif table_loc == "closest":
        speech_text = "Ok. Moving to the closest table."
        chosen_table = 1
    else:
        speech_text = "Alright. Let's do something else."

    set_intent_info(Intents.MOVE_TO_TABLE)

    return handler_input.response_builder.speak(speech_text).response


@sb.request_handler(can_handle_func=is_intent_name("HandFromGround"))
def hand_from_ground_intent_handler(handler_input):
    global num_objects

    set_intent_info(Intents.GET_OBJECTS)

    if num_objects == 1:
        speech_text = "Grabbing the object."
    if num_objects == 2:
        speech_text = (
            "I found "
            + str(num_objects)
            + " objects on the ground. Which one would you like?"
        )
        reprompt = "Which object would you like?"
        return (
            handler_input.response_builder.speak(speech_text)
            .ask(reprompt)
            .add_directive(
                ElicitSlotDirective(
                    updated_intent=Intent(name="ChooseObject"), slot_to_elicit="obj_loc"
                )
            )
            .response
        )
    else:
        speech_text = (
            "I found "
            + str(num_objects)
            + " objects on the ground. Which one would you like?"
        )
        reprompt = "Which object would you like?"
        handler_input.response_builder.ask(reprompt).add_directive(
            ElicitSlotDirective(
                updated_intent=Intent(name="ChooseObject"), slot_to_elicit="obj_loc"
            )
        )

    return handler_input.response_builder.speak(speech_text).response


@sb.request_handler(can_handle_func=is_intent_name("ChooseObject"))
def choose_object_intent_handler(handler_input):
    global chosen_object

    obj_loc = (
        handler_input.request_envelope.request.intent.slots["obj_loc"]
        .resolutions.resolutions_per_authority[0]
        .values[0]
        .value.name
    )
    print(obj_loc)

    if obj_loc == "right":
        speech_text = "Ok. Grabbing the right one."
        chosen_object = 0
    elif obj_loc == "left":
        speech_text = "Ok. Grabbing the left one."
        chosen_object = 1
    else:
        speech_text = "Alright. Let's do something else."

    set_intent_info(Intents.GRAB_FROM_GROUND)

    return handler_input.response_builder.speak(speech_text).response


@sb.request_handler(can_handle_func=is_intent_name("StopRobotIntent"))
def stop_robot_intent_handler(handler_input):
    set_intent_info(Intents.STOP)

    speech_text = "Stopping Hello Stretch!"

    return handler_input.response_builder.speak(speech_text).response


@sb.request_handler(can_handle_func=is_intent_name("StowIntent"))
def stow_intent_handler(handler_input):
    set_intent_info(Intents.STOW)
    speech_text = "Stowing Hello Stretch!"

    return handler_input.response_builder.speak(speech_text).response


@sb.request_handler(can_handle_func=is_intent_name("GetIntentsList"))
def get_intents_list_intent_handler(handler_input):
    global current_intent
    current_intent = "Hello Stretch Get Action List"


    # want_to_continue = handler_input.request_envelope.request.intent.slots['want_to_continue'].resolutions.resolutions_per_authority[0].values[0].value.name
    persistence_attr = handler_input.attributes_manager.persistent_attributes

    intent_choice = (
        handler_input.request_envelope.request.intent.slots["intent_choice"]
        .resolutions.resolutions_per_authority[0]
        .values[0]
        .value.name
    )
    print(intent_choice)

    match intent_choice:
        case "scan room":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="ScanRoom"))
            ).response
        case "pick up from the ground":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="HandFromGround"))
            ).response
        case "move to a table":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="MoveToTable"))
            ).response
        case "reach for a table":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="GrabFromTable"))
            ).response
        case "repeat":
            intent_choice = None
            # handler_input.attributes_manager.set_session_attributes(None)
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="GetIntentsList"))
            ).response
        case _:
            return handler_input.response_builder.speak("Ok. Goodbye")


@sb.request_handler(can_handle_func=is_intent_name("ChooseIntent"))
def choose_intent_handler(handler_input):

    intent_choice = (
        handler_input.request_envelope.request.intent.slots["intent_choice"]
        .resolutions.resolutions_per_authority[0]
        .values[0]
        .value.name
    )

    match intent_choice:
        case "scan room":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="ScanRoom"))
            ).response
        case "pick up from the ground":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="HandFromGround"))
            ).response
        case "move to a table":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="MoveToTable"))
            ).response
        case "reach for a table":
            return handler_input.response_builder.add_directive(
                DelegateDirective(updated_intent=Intent(name="GrabFromTable"))
            ).response
        case _:
            return handler_input.response_builder.speak("Ok. No action selected.")


@sb.request_handler(can_handle_func=is_intent_name("UserCustomAction"))
def user_custom_action_intent_handler(handler_input):
    global custom_intent, send_custom_intent

    custom_intent_num = int(handler_input.request_envelope.request.intent.slots['custom_intent_num'].value)
    print(custom_intent_num)

    # current_intent = "Hello Stretch " + custom_intent_array[custom_intent_num][0]

    speech = f"Ok. Running {custom_intent_array[(custom_intent_num - 1)][0]}."

    match custom_intent_num:
        case 1:
            set_intent_info(Intents.CUSTOM_1)
        case 2:
            set_intent_info(Intents.CUSTOM_2)
        case 3:
            set_intent_info(Intents.CUSTOM_3)

    send_custom_intent = True

    return handler_input.response_builder.speak(speech).response

@sb.request_handler(can_handle_func=is_intent_name("GrabCubeDemo"))
def grab_cube_demo_intent_handler(handler_input):
    global demo_cube_height

    set_intent_info(Intents.CUBE_DEMO)

    demo_cube_height = float(handler_input.request_envelope.request.intent.slots["cube_height"].value)

    if demo_cube_height > 1.1:
        demo_cube_height = 1.1
    elif demo_cube_height < 0:
        demo_cube_height = 0

    speech_text = (f"Okay, Stretch is moving the lift to {demo_cube_height} meters and getting ready to grab the cube. Please let me know when to close the gripper.")

    handler_input.response_builder.speak(speech_text).add_directive(ElicitSlotDirective(
                updated_intent=Intent(name="ChangeGripperState"), slot_to_elicit="gripper_state"
            ))

    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("ChangeGripperState"))
def change_gripper_state_intent_handler(handler_input):
    global close_gripper

    speech_text = "Closing the gripper!"

    state = handler_input.request_envelope.request.intent.slots['gripper_state'].resolutions.resolutions_per_authority[0].values[0].value.name

    if state == "close": 
        print("in1")
        close_gripper = True
        

    return handler_input.response_builder.speak(speech_text).response

# Create skill
skill_response = SkillAdapter(
    skill=sb.create(),
    skill_id="amzn1.ask.skill.061821fa-7468-4690-8a26-f559e7232188",
    app=app,
)

skill_response.register(app=app, route="/")

# --------------------------------------------------------------------
# Web Page Interactions
# --------------------------------------------------------------------

@app.route("/button_click", methods=["POST"])
def button_click():
    global current_intent, current_movement, send_custom_intent

    button_id = request.form.get("button_id")

    match button_id:
        case "scan_room":
            set_intent_info(Intents.SCAN_ROOM)
        case "grab_from_table":
            set_intent_info(Intents.REACH_TABLE)
        case "move_to_table":
            set_intent_info(Intents.GET_TABLES)
        case "grab_from_ground":
            set_intent_info(Intents.GET_OBJECTS)
        case "stop":
            set_intent_info(Intents.STOP)
        case "stow":
            set_intent_info(Intents.STOW)
        case "small_move_test":
            print("small move test")
            set_intent_info(Intents.TEST_LIFT_SMALL)
        case "custom_1":
            print("!")
            set_intent_info(Intents.CUSTOM_1)
            send_custom_intent = True
        case "grab_cube":
            print("grab cube")
            set_intent_info(Intents.CUBE_DEMO)

    return jsonify({"result": button_id})

@app.route("/custom_intent", methods=["POST"])
def custom_intent_builder():
    global custom_intent

    # adds movements to a custom intent list as buttons are pressed
    if custom_intent != None:
        id = request.form.get("button_id")
        print(id)

        if id == "Clear":
            name = custom_intent[0]
            custom_intent.clear()
            custom_intent.append(name)
        else:
            custom_intent.append(id)

        return jsonify({"result": id})
    else:
        result = "please choose an action"
        print(result)

        return jsonify({"result": result})


@app.route("/radio_selection", methods=["POST"])
def radio_selection():
    global custom_intent, custom_intent_array

    selected_option = request.form["radioOption"]
    # Perform server-side actions based on which radio button was selected
    match selected_option:
        case "option1":
            custom_intent = custom_intent_array[0]
        case "option2":
            custom_intent = custom_intent_array[1]
        case "option3":
            custom_intent = custom_intent_array[2]

    return jsonify({"result": selected_option})


def set_intent_info(intent_num):
    global current_intent, current_movement, node_data, custom_intent_num, demo_cube_height

    match intent_num:
        case Intents.STOP:
            intent_name = "Hello Stretch Stop"
        case Intents.STOW:
            intent_name = "Hello Stretch Stow"
        case Intents.SCAN_ROOM:
            intent_name = "Hello Stretch Scan Room"
            current_movement = "Rotates the wrist left or right to make a full circle"
        case Intents.REACH_TABLE:
            intent_name = "Hello Stretch Grab From Table"
            current_movement = "Move up, extend arm, rotate wrist, and close gripper"
        case Intents.MOVE_TO_TABLE:
            intent_name = "Hello Stretch Move To Table"
            current_movement = "Rotate the base, move forward, lift up, and rotate wrist"
        case Intents.GRAB_FROM_GROUND:
            intent_name = "Hello Stretch Hand From Ground"
            current_movement = "Rotate base, move forward, open gripper, move lift down, close gripper, move life up, rotate base, and move forward again to return to the start"
        case Intents.TEST_LIFT_SMALL:
            intent_name = "Testing Lift Movement"
        case Intents.CUSTOM_1:
            intent_name = "Custom Action 1"
            custom_intent_num = 0
        case Intents.CUSTOM_2:
            intent_name = "Custom Action 2"
            custom_intent_num = 1
        case Intents.CUSTOM_3:
            intent_name = "Custom Action 3"
            custom_intent_num = 2
        case Intents.CUBE_DEMO:
            intent_name = "Preparing To Grab a Cube -- Demo"
            current_movement = "Move lift to desired height and open gripper."
            demo_cube_height = .5

    current_intent = intent_name
    node_data["intent"] = intent_num.value
    print(intent_name)


# --------------------------------------------------------------------
# ROS2 Web Page Node
# --------------------------------------------------------------------
class WebPageNode(Node):
    def __init__(self):
        global send_custom_intent, custom_intent_num
        super().__init__("web_page_node")
        self.intent_publisher = self.create_publisher(Int32, "selected_intent_topic", 10)
        self.table_publisher = self.create_publisher(Int32, "selected_table_topic", 10)
        self.object_publisher = self.create_publisher(Int32, "selected_object_topic", 10)
        self.cube_height_publisher = self.create_publisher(Float32, "cube_demo_height", 10)
        self.custom_publisher = self.create_publisher(String, "custom_intent_topic", 10)
        self.gripper_state_publisher = self.create_publisher(Bool, "gripper_state_topic", 10)

        self.num_objects_subscribtion = self.create_subscription(
            Int32, "num_objects_topic", self.num_objects_callback, 10
        )
        self.num_tables_subscribtion = self.create_subscription(
            Int32, "num_tables_topic", self.num_tables_callback, 10
        )

        self.get_logger().info("Initialized")
        self.create_timer(1.0, self.publish_message)

    def num_objects_callback(self, msg):
        global num_objects

        self.get_logger().info('I heard: "%s"' % msg.data)
        num = int(f"{msg.data}")
        num_objects = num

    def num_tables_callback(self, msg):
        global num_tables

        self.get_logger().info('I heard: "%s"' % msg.data)
        num = int(f"{msg.data}")
        self.issue_alexa_command(num)
        num_tables = num

    def publish_message(self):
        global send_custom_intent

        self.publish_demo_cube_height()
        self.publish_gripper_state()
        # self.publish_chosen_object()
        # self.publish_chosen_table()

        self.publish_intent()

        if send_custom_intent is True:
            self.publish_custom_intent(custom_intent_num)
            send_custom_intent = False
    
    def publish_gripper_state(self):
        global close_gripper

        msg = Bool()
        msg.data = close_gripper

        if close_gripper is True:
            print("in2")
            self.gripper_state_publisher.publish(msg)
            close_gripper = False

    def publish_demo_cube_height(self):
        global demo_cube_height
        msg = Float32()
        if demo_cube_height >= 0:
            msg.data = demo_cube_height
            self.cube_height_publisher.publish(msg)
            self.get_logger().info(f"Publishing: {msg.data}")
            demo_cube_height = -1

    def publish_intent(self):
        global node_data
        msg = Int32()
        if node_data["intent"] != 1111:
            msg.data = node_data["intent"]
            self.intent_publisher.publish(msg)
            self.get_logger().info(f"Publishing: {msg.data}")
            node_data["intent"] = 1111

    def publish_chosen_object(self):
        global chosen_object
        msg = Int32()
        if chosen_object >= 0:
            msg.data = chosen_object
            self.object_publisher.publish(msg)
            self.get_logger().info(f"Publishing num_objects: {msg.data}")
            chosen_object = -1

    def publish_chosen_table(self):
        global chosen_table
        msg = Int32()
        if chosen_table >= 0:
            msg.data = chosen_table
            self.table_publisher.publish(msg)
            self.get_logger().info(f"Publishing num_tables: {msg.data}")
            chosen_table = -1

    def publish_custom_intent(self, num):
        global custom_intent_array
        print("2")
        msg = String()

        # print(custom_intent_array[num])

        if custom_intent_array[num] is not "":
            msg.data = ", ".join(custom_intent_array[num])
            print(msg.data)
            self.custom_publisher.publish(msg)
            self.get_logger().info(f"Publishing custom action " + str(num))

# --------------------------------------------------------------------
# Runs node and Flask server
# --------------------------------------------------------------------

def run_ros2_node():
    rclpy.init()
    node = WebPageNode()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()


def run_flask_server():
    print("running flask")
    app.run(port=PORT)


def main():
    flask_thread = threading.Thread(target=run_flask_server)
    flask_thread.start()

    run_ros2_node()


if __name__ == "__main__":
    main()
