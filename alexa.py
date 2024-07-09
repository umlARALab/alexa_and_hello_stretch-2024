import ngrok

from flask import Flask, render_template, jsonify
from flask_ask_sdk.skill_adapter import SkillAdapter
from flask import request
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.dialog import (ElicitSlotDirective, DelegateDirective)
from ask_sdk_model import (Response, IntentRequest, DialogState, SlotConfirmationStatus, Slot, ui, IntentConfirmationStatus)
from ask_sdk_model.ui import SimpleCard
from ask_sdk_model.intent import Intent

from util import Intents
# from ask_sdk_core.attributes_manager import AttributesManager

# import rclpy
# from rclpy.node import Node 
# from rclpy.action import ActionClient

# action_client = ActionClient(Node("alexa_interface"))


# still need to fix custom intent lists ==> maybe make just one for testing???
# have intent appear on page as user makes it
# have alexa respond to it
# then figure out stretch thing!!!!!


from objects import *
# from stretch import *

PORT = 9999
# ngrok http 9999 to start
# python alexa.py

app = Flask(__name__)

sb = SkillBuilder()

current_intent = ""
current_movement = ""
custom_intent = None
custom_intent_array = [["Custom Action 1"], ["Custom Action 2"], ["Custom Action 3"]]

@app.route("/")
def homepage():
    global current_intent, custom_intent, current_movement

    custom_text = "Choose a Custom Action to build!"

    print(current_movement)

    if custom_intent is not None:
        custom_text = ", ".join(custom_intent)
        
    return render_template('index.html', intent = current_intent, movements = current_movement, custom = custom_text)

@sb.request_handler(can_handle_func=is_request_type("LaunchRequest"))
def launch_request_handler(handler_input):
    global current_intent
    current_intent = "Launch Request"

    # type: (HandlerInput) -> Response
    speech_text = "Welcome to the Alexa Skills Kit, you can say hello!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)).set_should_end_session(
        False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("HelloWorldIntent"))
def hello_world_intent_handler(handler_input):
    global current_intent
    current_intent = "Hello World Intent"

    # type: (HandlerInput) -> Response
    speech_text = "Hello World!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)).set_should_end_session(
        True)
    
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("AMAZON.HelpIntent"))
def help_intent_handler(handler_input):
    global current_intent
    current_intent = "Amazon Help Intent"

    # type: (HandlerInput) -> Response
    speech_text = "You can say hello to me!"

    handler_input.response_builder.speak(speech_text).ask(speech_text).set_card(
        SimpleCard("Hello World", speech_text))
    return handler_input.response_builder.response

@sb.request_handler(
    can_handle_func=lambda handler_input :
        is_intent_name("AMAZON.CancelIntent")(handler_input) or
        is_intent_name("AMAZON.StopIntent")(handler_input))
def cancel_and_stop_intent_handler(handler_input):
    global current_intent
    current_intent = "Amazon Cancel or Stop Intent"

    # type: (HandlerInput) -> Response
    speech_text = "Goodbye!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)).set_should_end_session(
            True)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_request_type("SessionEndedRequest"))
def session_ended_request_handler(handler_input):
    global current_intent
    current_intent = "Session End Request"

    # type: (HandlerInput) -> Response
    # any cleanup logic goes here

    return handler_input.response_builder.response

@sb.exception_handler(can_handle_func=lambda i, e: True)
def all_exception_handler(handler_input, exception):
    global current_intent
    current_intent = "Exception Handler"

    # type: (HandlerInput, Exception) -> Response
    # Log the exception in CloudWatch Logs
    print(exception)

    speech = "Sorry, I didn't get it. Can you please say it again!!"
    handler_input.response_builder.speak(speech).ask(speech)
    return handler_input.response_builder.response


##### HELLO STRETCH INTENT HANDLERS #####

@sb.request_handler(can_handle_func=is_intent_name("ScanRoom"))
def scan_room_intent_handler(handler_input):

    set_intent_info(Intents.SCAN_ROOM)

    # type: (HandlerInput) -> Response
    speech_text = "Scanning the room!"

    #scan_room()

    handler_input.response_builder.speak(speech_text)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("GrabFromTable"))
def grab_from_table_intent_handler(handler_input):

    set_intent_info(Intents.GRAB_FROM_GROUND)

    print(current_movement)

    # type: (HandlerInput) -> Response
    speech_text = "Grab from table!"

    #reach_table()

    handler_input.response_builder.speak(speech_text)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("MoveToTable"))
def move_to_table_intent_handler(handler_input):

    set_intent_info(Intents.MOVE_TO_TABLE)

    table_loc = handler_input.request_envelope.request.intent.slots['table_loc'].value

    if num_tables == 1:
        #table = choose_table(0)
        #move_to_table(table[0], table[1])
        speech_text = "I only see one table. Moving to the table."
        return handler_input.response_builder.speak(speech_text).response
    elif table_loc == None:
        if num_tables == 2:
            print(2)
            speech_text = (
                "I found " + str(num_tables) + " tables. Would you like the closest or farthest?")
            reprompt = "Would you like the closest or farthest?"
            # handler_input.response_builder.speak(speech_text).add_directive(DelegateDirective(updated_intent=Intent(name="ChooseTable")))
            return handler_input.response_builder.speak(speech_text).ask(reprompt).add_directive(ElicitSlotDirective(updated_intent=Intent(name="ChooseTable"), slot_to_elicit="table_loc")).response
        else:
            print(num_tables)
            speech_text = (
                "I found " + str(num_tables) + " tables. Would you like the closest, farthest, or another one?")
            reprompt = "Would you like the closest, farthest, or another one?"
            handler_input.response_builder.ask(reprompt).add_directive(ElicitSlotDirective(updated_intent=Intent(name="ChooseTable"), slot_to_elicit="table_loc"))
    else:
        if table_loc == "farthest":
            speech_text = "Ok. Moving to the farthest table."
            #table = choose_table(0)
        elif table_loc == "closest":
            speech_text = "Ok. Moving to the closest table."
            #table = choose_table(num_tables)

    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("ChooseTable"))
def choose_table_intent_handler(handler_input):

    # type: (HandlerInput) -> Response
    
    table_loc = handler_input.request_envelope.request.intent.slots['table_loc'].resolutions.resolutions_per_authority[0].values[0].value.name

    speech_text = ("There are tables.")

    if table_loc == "farthest":
        speech_text = "Ok. Moving to the farthest table."
        #table = choose_table(0)
    elif table_loc == "closest":
        speech_text = "Ok. Moving to the closest table."
        #table = choose_table(num_tables)
    else:
        speech_text = "Alright. Let's do something else."
    
    #move_to_table(table[0], table[1])
    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("HandFromGround"))
def hand_from_ground_intent_handler(handler_input):
    
    set_intent_info(Intents.GRAB_FROM_GROUND)

    if num_objects == 1:
        speech_text = "Grabbing the object."
    if num_objects == 2:
        print(2)
        speech_text = (
            "I found " + str(num_objects) + " objects on the ground. Which one would you like?")
        reprompt = "Which object would you like?"
        return handler_input.response_builder.speak(speech_text).ask(reprompt).add_directive(ElicitSlotDirective(updated_intent=Intent(name="ChooseObject"), slot_to_elicit="obj_loc")).response
    else:
        print(num_objects)
        speech_text = (
            "I found " + str(num_objects) + " objects on the ground. Which one would you like?")
        reprompt = "Which object would you like?"
        handler_input.response_builder.ask(reprompt).add_directive(ElicitSlotDirective(updated_intent=Intent(name="ChooseObject"), slot_to_elicit="obj_loc"))

    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("ChooseObject"))
def choose_object_intent_handler(handler_input):
    # type: (HandlerInput) -> Response
    
    obj_loc = handler_input.request_envelope.request.intent.slots['obj_loc'].resolutions.resolutions_per_authority[0].values[0].value.name
    print(obj_loc)

    if obj_loc == "right":
        speech_text = "Ok. Grabbing the right one."
    elif obj_loc == "left":
        speech_text = "Ok. Grabbing the left one."
    else:
        speech_text = "Alright. Let's do something else."
    
    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("StopRobotIntent"))
def stop_robot_intent_handler(handler_input):
    set_intent_info(Intents.STOP)

    # type: (HandlerInput) -> Response
    
    speech_text = "Stopping Hello Stretch!"

    #stop()
    
    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("StowIntent"))
def stow_intent_handler(handler_input):
    set_intent_info(Intents.STOW)

    # type: (HandlerInput) -> Response
    
    speech_text = "Stowing Hello Stretch!"

    #stow()
    
    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("GetIntentsList"))
def get_intents_list_intent_handler(handler_input):
    global current_intent
    current_intent = "Hello Stretch Get Action List"

    # type: (HandlerInput) -> Response

    # want_to_continue = handler_input.request_envelope.request.intent.slots['want_to_continue'].resolutions.resolutions_per_authority[0].values[0].value.name
    persistence_attr = handler_input.attributes_manager.persistent_attributes

    intent_choice = handler_input.request_envelope.request.intent.slots['intent_choice'].resolutions.resolutions_per_authority[0].values[0].value.name
    print(intent_choice)

    if intent_choice == "scan room":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="ScanRoom"))).response
    elif intent_choice == "pick up from the ground":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="HandFromGround"))).response
    elif intent_choice == "move to a table":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="MoveToTable"))).response
    elif intent_choice == "reach for a table":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="GrabFromTable"))).response
    elif intent_choice == "repeat":
        intent_choice = None
        #handler_input.attributes_manager.set_session_attributes(None)
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="GetIntentsList"))).response
    else:
        return handler_input.response_builder.speak("Ok. Goodbye")

@sb.request_handler(can_handle_func=is_intent_name("ChooseIntent"))
def choose_intent_handler(handler_input):
    # type: (HandlerInput) -> Response
    
    intent_choice = handler_input.request_envelope.request.intent.slots['intent_choice'].resolutions.resolutions_per_authority[0].values[0].value.name

    if intent_choice == "scan room":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="ScanRoom"))).response
    elif intent_choice == "pick up from the ground":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="HandFromGround"))).response
    elif intent_choice == "move to a table":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="MoveToTable"))).response
    elif intent_choice == "reach for a table":
        return handler_input.response_builder.add_directive(DelegateDirective(updated_intent=Intent(name="GrabFromTable"))).response
    else:
        return handler_input.response_builder.speak("Ok. No action selected.")

@sb.request_handler(can_handle_func=is_intent_name("UserCustomAction"))
def user_custom_action_intent_handler(handler_input):
    # global current_intent, custom_intent

    # custom_intent_num = handler_input.request_envelope.request.intent.slots['custom_intent_num']
    # print(custom_intent_num)
    speech = "I'm sorry, there are only three custom actions available."

    # if custom_intent is not None:
    #     custom_intent_num = custom_intent_num.resolutions.resolutions_per_authority[0].values[0].value.name
    #     print(custom_intent_num)

    #     current_intent = "Hello Stretch " + custom_intent_array[custom_intent_num][0]

    #     if custom_intent_num == 1:
    #         speech = "Ok. Running Custom Action 1."
    #     elif custom_intent_num == 2:
    #         speech = "Ok. Running Custom Action 2."
    #     elif custom_intent_num == 3:
    #         speech = "Ok. Running Custom Action 3."
    #     else:
    #         speech = "I'm sorry, there are only three custom actions available."

    return handler_input.response_builder.speak(speech).response

skill_response = SkillAdapter(
    skill=sb.create(), skill_id="amzn1.ask.skill.061821fa-7468-4690-8a26-f559e7232188", app=app)

skill_response.register(app=app, route="/")

###### Page interactions

@app.route('/button_click', methods=['POST'])
def button_click():
    global current_intent, current_movement

    button_id = request.form.get('button_id')

    if "scan_room" == button_id:
        set_intent_info(Intents.SCAN_ROOM)
    elif "grab_from_table" == button_id:
        set_intent_info(Intents.REACH_TABLE)
    elif "move_to_table" == button_id:
        set_intent_info(Intents.MOVE_TO_TABLE)
    elif "grab_from_ground" == button_id:
        set_intent_info(Intents.GRAB_FROM_GROUND)
    elif "stop" == button_id:
        set_intent_info(Intents.STOP)
    elif "stow" == button_id:
        set_intent_info(Intents.STOW)

    return jsonify({'result': button_id})

@app.route('/custom_intent', methods=['POST'])
def custom_intent_builder():
    global custom_intent

    if custom_intent != None:
        id = request.form.get('button_id')
        print(id)

        if id == "clear":
            name = custom_intent[0]
            custom_intent.clear()
            custom_intent.append(name)
        else:
            custom_intent.append(id)

        return jsonify({'result': id})
    else:
        result = "please choose an action"
        print(result)

        return jsonify({'result': result})
    
   
 
@app.route('/radio_selection', methods=['POST'])
def radio_selection():
    global custom_intent, custom_intent_array

    selected_option = request.form['radioOption']
    # Perform server-side actions based on which radio button was selected
    if selected_option == 'option1':
        custom_intent = custom_intent_array[0]
    elif selected_option == 'option2':
        custom_intent = custom_intent_array[1]
    elif selected_option == 'option3':
        custom_intent = custom_intent_array[2]

    return jsonify({'result': selected_option})

def set_intent_info(intent_num):
    global current_intent, current_movement

    if intent_num == Intents.STOP:
        current_intent = "Hello Stretch Stop"
    elif intent_num == Intents.STOW:
        current_intent = "Hello Stretch Stow"
    elif intent_num == Intents.SCAN_ROOM:
        current_intent = "Hello Stretch Scan Room"
        current_movement = "Rotates the wrist left or right to make a full circle"
    elif intent_num == Intents.REACH_TABLE:
        current_intent = "Hello Stretch Grab From Table"
        current_movement = "Move up, extend arm, rotate wrist, and close gripper"
    elif intent_num == Intents.MOVE_TO_TABLE:
        current_intent = "Hello Stretch Move To Table"
        current_movement = "Rotate the base, move forward, lift up, and rotate wrist"
    elif intent_num == Intents.GRAB_FROM_GROUND:
        current_intent = "Hello Stretch Hand From Ground"
        current_movement = "Rotate base, move forward, open gripper, move lift down, close gripper, move life up, rotate base, and move forward again to return to the start"

if __name__ == '__main__':
    app.run(port = PORT, debug=True)
