import json
from typing import Dict, Any, List

import rospy
from smach import State, UserData, StateMachine

from context import Context
from lasr_rasa.srv import Rasa, RasaRequest
from lasr_skills import Listen
from states.speak import Speak


class TakeOrder(StateMachine):

    def __init__(self, context: Context):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "ASK_FOR_ORDER",
                Speak("What would you like to order?"),
                transitions={"success": "LISTEN_FOR_ORDER"}
            )
            if context.mic_input:
                self.add(
                    "LISTEN_FOR_ORDER",
                    Listen(),
                    transitions={
                        "succeeded": "PARSE_ORDER",
                        "aborted": "REPEAT_ORDER",
                        "preempted": "REPEAT_ORDER",
                    },
                    remapping={"sequence": "order"},
                )
            else:
                self.add(
                    "LISTEN_FOR_ORDER",
                    TakeOrder.ListenForOrder(),
                    transitions={"success": "PARSE_ORDER"},
                )
            self.add(
                "PARSE_ORDER",
                TakeOrder.ParseOrder(context),
                transitions={
                    "success": "success",
                    "failure": "REPEAT_ORDER",
                },
            )
            self.add(
                "REPEAT_ORDER",
                Speak("I'm sorry, I didn't catch that. Could you repeat your order?"),
                transitions={"success": "LISTEN_FOR_ORDER"},
            )

    class ListenForOrder(State):

        def __init__(self):
            super().__init__(outcomes=["success"], output_keys=["order"])

        def execute(self, userdata: UserData) -> str:
            userdata["order"] = "I would like chips"
            return "success"

    class ParseOrder(State):

        def __init__(self, context: Context):
            super().__init__(outcomes=["success", "failure"], input_keys=["order"])
            self.rasa_service = rospy.ServiceProxy("/lasr_rasa/parse", Rasa)
            self.rasa_service.wait_for_service()
            self.context: Context = context

        def execute(self, userdata: UserData) -> str:
            order: str = userdata["order"]
            request = RasaRequest(order)
            response = self.rasa_service(request)
            json_string: str = response.json_response
            # noinspection PyBroadException
            try:
                json_object: Dict[str, Any] = json.loads(json_string)
                intent: Dict[str, Any] = json_object["intent"]
                intent_name: str = intent["name"]
                if intent_name == "fav_drink":
                    entities: Dict[str, List[Dict[str, Any]]] = json_object["entities"]
                    name_object = entities["name"][0]
                    self.context.order = name_object["value"]
                    return "success"
            except Exception:
                pass
            return "failure"
