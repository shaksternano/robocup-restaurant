from context import Context
from smach import State, UserData, StateMachine

from lasr_skills import Say


class ConfirmOrder(StateMachine):

    def __init__(self, context: Context):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "PROCESS_ORDER",
                ConfirmOrder.ProcessOrder(context),
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
        def __init__(self, context: Context):
            super().__init__(
                outcomes=["success"],
                output_keys=["text"],
            )
            self.context: Context = context

        def execute(self, userdata: UserData) -> str:
            userdata["text"] = "I will bring you some " + self.context.order
            return "success"
