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
        angle = math.atan2(table_y - current_y, table_x - current_x)

        pose = Pose()
        pose.position.x = table_x
        pose.position.y = table_y
        pose.orientation.z = math.sin(angle / 2)
        pose.orientation.w = math.cos(angle / 2)
        self.controller.change_pose(pose, prevent_crashes=True)
        return "success"
