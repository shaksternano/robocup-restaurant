from lasr_skills import Say
from smach import State, UserData, StateMachine


class TakeOrder(StateMachine):

    def __init__(self):
        super().__init__(outcomes=["success"], output_keys=["order"])
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
                TakeOrder.ListenForOrder(),
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

        def __init__(self):
            super().__init__(outcomes=["success"], output_keys=["order"])

        def execute(self, userdata: UserData) -> str:
            userdata["order"] = "chips"
            return "success"
