import math

from geometry_msgs.msg import Pose
from smach import State, UserData
from tiago_controller import TiagoController


class MoveToTable(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success"],
            input_keys=["table_x", "table_y"],
        )
        self.controller: TiagoController = controller

    def execute(self, userdata: UserData) -> str:
        table_x: float = userdata["table_x"]
        table_y: float = userdata["table_y"]
        current_x, current_y, _ = self.controller.get_current_pose()

        distance = math.sqrt((table_x - current_x) ** 2 + (table_y - current_y) ** 2)
        new_distance = distance - 2
        angle = math.atan2(table_y - current_y, table_x - current_x)
        travel_x = current_x + new_distance * math.cos(angle)
        travel_y = current_y + new_distance * math.sin(angle)

        pose = Pose()
        pose.position.x = travel_x
        pose.position.y = travel_y
        pose.orientation.z = math.sin(angle / 2)
        pose.orientation.w = math.cos(angle / 2)
        self.controller.change_pose(pose)
        return "success"
