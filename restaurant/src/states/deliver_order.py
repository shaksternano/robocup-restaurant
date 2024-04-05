from smach import State, UserData, StateMachine

from context import Context
from states.move_to_point import MoveToPoint
from tiago_controller import TiagoController


class DeliverOrder(StateMachine):

    def __init__(self, controller: TiagoController, context: Context):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "SET_CUSTOMER_POSITION",
                DeliverOrder.SetCustomerPosition(context),
                transitions={"success": "RETURN_TO_CUSTOMER"},
            )
            self.add(
                "RETURN_TO_CUSTOMER",
                MoveToPoint(controller),
                transitions={"failure": "success"},
            )

    class SetCustomerPosition(State):

        def __init__(self, context: Context):
            super().__init__(outcomes=["success"], output_keys=["x", "y"])
            self.context: Context = context

        def execute(self, userdata: UserData) -> str:
            userdata["x"] = self.context.customer_x
            userdata["y"] = self.context.customer_y
            return "success"
