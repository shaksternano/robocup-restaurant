from context import Context
from smach import State, UserData, StateMachine

from states.speak import Speak


class RequestOrder(StateMachine):

    def __init__(self, context: Context):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "ASK_FOR_ORDER",
                Speak("What would you like to order?"),
                transitions={"success": "LISTEN_FOR_ORDER"}
            )
            self.add(
                "LISTEN_FOR_ORDER",
                RequestOrder.ListenForOrder(context),
            )

    class ListenForOrder(State):

        def __init__(self, context: Context):
            super().__init__(outcomes=["success"])
            self.context: Context = context

        def execute(self, userdata: UserData) -> str:
            self.context.order = "chips"
            return "success"
