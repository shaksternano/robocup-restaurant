from smach import State, UserData

from tiago_controller import TiagoController


class CheckForPerson(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["person", "no_person"],
            input_keys=["robot_table_x", "robot_table_y", "table_orientation"],
            output_keys=["robot_table_x", "robot_table_y", "table_orientation"],
        )
        self.controller: TiagoController = controller

    def execute(self, userdata: UserData) -> str:
        detections = self.controller.get_detections()
        for detection in detections:
            if detection.name == "person":
                return "person"
        return "no_person"
