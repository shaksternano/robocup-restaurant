import math

from geometry_msgs.msg import Point
from smach import State, UserData
from tiago_controller import TiagoController


class LocateTable(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success", "end"],
            output_keys=["table_x", "table_y", "table_orientation"],
        )
        self.controller: TiagoController = controller
        self.table_index: int = 0

    def execute(self, userdata: UserData) -> str:
        table_position = self.locate_table()
        if table_position.x == 0 and table_position.y == 0:
            return "end"
        userdata["table_x"] = table_position.x
        userdata["table_y"] = table_position.y
        userdata["table_orientation"] = math.pi
        return "success"

    def locate_table(self) -> Point:
        turn_times = 8
        rotation_angle = 2 * math.pi / turn_times
        for i in range(turn_times):
            detections = self.controller.get_detections()
            for detection in detections:
                if "person" in detection.name:
                    return detection.point
            self.controller.rotate(rotation_angle)
        return Point()
