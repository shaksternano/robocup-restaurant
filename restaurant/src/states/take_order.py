import random
from typing import List

from smach import State, UserData, StateMachine

from context import Context
from lasr_skills import Listen
from states.speak import Speak
from tiago_controller import TiagoController


class TakeOrder(StateMachine):

    def __init__(self, controller: TiagoController, context: Context):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "ASK_FOR_ORDER",
                Speak("What would you like to order?"),
                transitions={"success": "LISTEN_FOR_ORDER"},
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
                TakeOrder.ParseOrder(controller, context),
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
            self.example_orders: List[str] = [
                "I would like chips",
                "I would like a burger",
                "I would like a cola",
                "I want chips",
                "I want a burger",
                "I want a cola",
                "Give me chips",
                "Give me a burger",
                "Give me a cola",
            ]

        def execute(self, userdata: UserData) -> str:
            userdata["order"] = random.choice(self.example_orders)
            return "success"

    class ParseOrder(State):

        def __init__(self, controller: TiagoController, context: Context):
            super().__init__(outcomes=["success", "failure"], input_keys=["order"])
            self.controller: TiagoController = controller
            self.context: Context = context

        def execute(self, userdata: UserData) -> str:
            order: str = userdata["order"]
            order_item = self.controller.get_order_item(order)
            if order_item:
                self.context.order = order_item
                return "success"
            else:
                return "failure"
