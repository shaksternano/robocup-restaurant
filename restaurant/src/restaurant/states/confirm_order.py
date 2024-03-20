from lasr_skills import Say
from smach import State, UserData, StateMachine


class ConfirmOrder(StateMachine):

    def __init__(self):
        super().__init__(outcomes=["success"], input_keys=["order"], output_keys=["order"])
        with self:
            self.add(
                "PROCESS_ORDER",
                ConfirmOrder.ProcessOrder(),
                transitions={"success": "SAY"},
            )
            self.add(
                "SAY",
                Say(),
                transitions={
                    "succeeded": "success",
                    "aborted": "success",
                    "preempted": "success",
                },
            )

    class ProcessOrder(State):
        def __init__(self):
            super().__init__(
                outcomes=["success"],
                input_keys=["order"],
                output_keys=["order", "text"],
            )

        def execute(self, userdata: UserData) -> str:
            userdata["text"] = "I will bring you some " + userdata["order"]
            return "success"
