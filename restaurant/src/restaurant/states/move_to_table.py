import math

from geometry_msgs.msg import Pose
from smach import State, UserData
from tiago_controller import TiagoController


class MoveToTable(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success"],
            input_keys=["table_x", "table_y", "table_orientation"],
        )
        self.controller: TiagoController = controller

    def execute(self, userdata: UserData) -> str:
        table_x = userdata["table_x"]
        table_y = userdata["table_y"]
        table_orientation = userdata["table_orientation"]
        pose = Pose()
        pose.position.x = table_x
        pose.position.y = table_y
        pose.orientation.z = math.sin(table_orientation / 2)
        pose.orientation.w = math.cos(table_orientation / 2)
        self.controller.change_pose(pose)
        return "success"
