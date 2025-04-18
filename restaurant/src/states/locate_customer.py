from smach import State, UserData

from context import Context
from tiago_controller import TiagoController


class LocateCustomer(State):

    def __init__(self, controller: TiagoController, context: Context):
        super().__init__(
            outcomes=["success", "failure"],
            output_keys=["x", "y"],
        )
        self.controller: TiagoController = controller
        self.context: Context = context

    def execute(self, userdata: UserData) -> str:
        customer_position = self.controller.locate_person()
        if customer_position.x == 0 and customer_position.y == 0:
            return "failure"
        userdata["x"] = customer_position.x
        userdata["y"] = customer_position.y
        self.context.customer_x = customer_position.x
        self.context.customer_y = customer_position.y
        return "success"
