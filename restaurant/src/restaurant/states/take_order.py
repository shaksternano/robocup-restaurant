from context import Context
from smach import State, UserData, StateMachine

from lasr_skills import Say


class TakeOrder(StateMachine):

    def __init__(self, context: Context):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "PREPARE_SPEECH",
                TakeOrder.PrepareSpeech(),
                transitions={"success": "SAY"}
            )
            self.add(
                "SAY",
                Say(),
                transitions={
                    "succeeded": "LISTEN_FOR_ORDER",
                    "aborted": "LISTEN_FOR_ORDER",
                    "preempted": "LISTEN_FOR_ORDER",
                },
            )
            self.add(
                "LISTEN_FOR_ORDER",
                TakeOrder.ListenForOrder(context),
            )

    class PrepareSpeech(State):

        def __init__(self):
            super().__init__(
                outcomes=["success"],
                output_keys=["text"],
            )

        def execute(self, userdata: UserData) -> str:
            userdata["text"] = "What would you like to order?"
            return "success"

    class ListenForOrder(State):

        def __init__(self, context: Context):
            super().__init__(outcomes=["success"])
            self.context: Context = context

        def execute(self, userdata: UserData) -> str:
            self.context.order = "chips"
            return "success"
