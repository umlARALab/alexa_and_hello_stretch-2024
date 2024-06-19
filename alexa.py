import ngrok

from flask import Flask
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model.dialog import (ElicitSlotDirective, DelegateDirective)
from ask_sdk_model import (Response, IntentRequest, DialogState, SlotConfirmationStatus, Slot, ui, IntentConfirmationStatus)
from ask_sdk_model.ui import SimpleCard
from ask_sdk_model.intent import Intent

from tables import *
from robot import *

PORT = 9999
# ngrok http 9999 to start
# python alexa.py

app = Flask(__name__)

sb = SkillBuilder()

@app.route("/")
def homepage():
    return "Welcome!"

@sb.request_handler(can_handle_func=is_request_type("LaunchRequest"))
def launch_request_handler(handler_input):
    # type: (HandlerInput) -> Response
    speech_text = "Welcome to the Alexa Skills Kit, you can say hello!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)).set_should_end_session(
        False)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("HelloWorldIntent"))
def hello_world_intent_handler(handler_input):
    # type: (HandlerInput) -> Response
    speech_text = "Hello World!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)).set_should_end_session(
        True)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("AMAZON.HelpIntent"))
def help_intent_handler(handler_input):
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
    # type: (HandlerInput) -> Response
    speech_text = "Goodbye!"

    handler_input.response_builder.speak(speech_text).set_card(
        SimpleCard("Hello World", speech_text)).set_should_end_session(
            True)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_request_type("SessionEndedRequest"))
def session_ended_request_handler(handler_input):
    # type: (HandlerInput) -> Response
    # any cleanup logic goes here

    return handler_input.response_builder.response

@sb.exception_handler(can_handle_func=lambda i, e: True)
def all_exception_handler(handler_input, exception):
    # type: (HandlerInput, Exception) -> Response
    # Log the exception in CloudWatch Logs
    print(exception)

    speech = "Sorry, I didn't get it. Can you please say it again!!"
    handler_input.response_builder.speak(speech).ask(speech)
    return handler_input.response_builder.response


##### HELLO STRETCH INTENT HANDLERS #####

@sb.request_handler(can_handle_func=is_intent_name("ScanRoom"))
def scan_room_intent_handler(handler_input):
    # type: (HandlerInput) -> Response
    speech_text = "Scanning the room!"

    scan_room()

    handler_input.response_builder.speak(speech_text)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("GrabFromTable"))
def grab_obj_intent_handler(handler_input):
    # type: (HandlerInput) -> Response
    speech_text = "Grab from table!"

    reach_table()

    handler_input.response_builder.speak(speech_text)
    return handler_input.response_builder.response

@sb.request_handler(can_handle_func=is_intent_name("MoveToTable"))
def move_to_table_intent_handler(handler_input):

    if num_tables == 1:
        table = choose_table(0)
        move_to_table(table[0], table[1])
        speech_text = "I only see one table. Moving to the table."
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
            "I found " + str(num_tables) + " tables. Would you like the closest, farthest, or one in the middle?")
        reprompt = "Would you like the closest, farthest, or one in the middle?"
        handler_input.response_builder.ask(reprompt).add_directive(ElicitSlotDirective(updated_intent=Intent(name="ChooseTable"), slot_to_elicit="table_loc"))

    return handler_input.response_builder.speak(speech_text).response

@sb.request_handler(can_handle_func=is_intent_name("ChooseTable"))
def choose_table_intent_handler(handler_input):
    # type: (HandlerInput) -> Response
    # speech_text = "Grab from table!"

    current_intent = handler_input.request_envelope.request.intent

    table_loc = handler_input.request_envelope.request.intent.slots['table_loc'].value

    speech_text = ("There are tables.")

    if table_loc == "middle" or table_loc == "one in the middle":
        make_middle_tables()
        if num_tables == 1:
            table = choose_table(0)
            speech_text = "Ok. Moving to the table."
        elif num_tables == 2:
            speech_text = (
                "There are " + str(num_tables) + " tables. Would you like the closest or farthest?")
            reprompt = "Would you like the closest or farthest?"
            handler_input.response_builder.ask(reprompt).add_directive(ElicitSlotDirective(slot_to_elicit="table_loc"))
        else:
            speech_text = (
                "There are " + str(num_tables) + " tables. Would you like the closest, farthest, or other?")
            reprompt = "Would you like the closest, farthest, or other?"
            handler_input.response_builder.ask(reprompt).add_directive(ElicitSlotDirective(slot_to_elicit="table_loc"))
    elif table_loc == "farthest":
        speech_text = "Ok. Moving to the farthest table."
        table = choose_table(0)
    elif table_loc == "closest":
        speech_text = "Ok. Moving to the closest table."
        table = choose_table(num_tables)
    
    
    move_to_table(table[0], table[1])
    return handler_input.response_builder.speak(speech_text).response


skill_response = SkillAdapter(
    skill=sb.create(), skill_id="amzn1.ask.skill.061821fa-7468-4690-8a26-f559e7232188", app=app)

skill_response.register(app=app, route="/")

if __name__ == '__main__':
    app.run(port = PORT, debug=True)
