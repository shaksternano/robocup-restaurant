from smach import State, UserData

from tiago_controller import TiagoController


class LookAtTable(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success"],
            input_keys=["table_x", "table_y", "robot_table_x", "robot_table_y", "table_orientation"],
            output_keys=["robot_table_x", "robot_table_y", "table_orientation"],
        )
        self.controller: TiagoController = controller

    def execute(self, userdata: UserData) -> str:
        table_x: float = userdata["table_x"]
        table_y: float = userdata["table_y"]
        self.controller.look_straight()
        self.controller.turn_to_face(table_x, table_y)
        return "success"
