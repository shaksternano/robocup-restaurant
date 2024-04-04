from smach import State, UserData
from tiago_controller import TiagoController


class LocateStaff(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success"],
            output_keys=["x", "y"],
        )
        self.controller: TiagoController = controller
        self.table_index: int = 0

    def execute(self, userdata: UserData) -> str:
        staff_position = self.controller.locate_person()
        userdata["x"] = staff_position.x
        userdata["y"] = staff_position.y
        return "success"
