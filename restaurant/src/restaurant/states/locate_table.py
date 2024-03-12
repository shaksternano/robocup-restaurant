import math
from typing import List

from smach import State, UserData
from tiago_controller import TiagoController


class Position:

    def __init__(self, x: float, y: float, orientation: float = 0):
        self.x: float = x
        self.y: float = y
        self.orientation: float = orientation


dining_area_entrance: Position = Position(1.4, -4.5)

table_positions: List[Position] = [
    Position(-1, -6.8),
    Position(-1, -8.8),
    Position(-1, -10.8),
]

table_front_positions: List[Position] = [
    Position(1, -6.8, math.pi),
    Position(1, -8.8, math.pi),
    Position(1, -10.8, math.pi),
]


class LocateTable(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success", "end"],
            output_keys=["table_x", "table_y", "robot_table_x", "robot_table_y", "table_orientation"],
        )
        self.controller: TiagoController = controller
        self.table_index: int = 0

    def execute(self, userdata: UserData) -> str:
        self.controller.look_straight()
        if self.table_index >= len(table_positions):
            return "end"
        table_position = table_positions[self.table_index]
        table_front_position = table_front_positions[self.table_index]
        userdata["table_x"] = table_position.x
        userdata["table_y"] = table_position.y
        userdata["robot_table_x"] = table_front_position.x
        userdata["robot_table_y"] = table_front_position.y
        userdata["table_orientation"] = table_front_position.orientation
        self.table_index += 1
        return "success"
