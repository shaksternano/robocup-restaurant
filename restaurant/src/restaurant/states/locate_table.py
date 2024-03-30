from smach import State, UserData
from tiago_controller import TiagoController


class LocateTable(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success", "end"],
            output_keys=["x", "y"],
        )
        self.controller: TiagoController = controller
        self.table_index: int = 0

    def execute(self, userdata: UserData) -> str:
        table_position = self.controller.locate_person()
        if table_position.x == 0 and table_position.y == 0:
            return "end"
        userdata["x"] = table_position.x
        userdata["y"] = table_position.y
        return "success"
