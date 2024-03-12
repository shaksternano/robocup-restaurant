from geometry_msgs.msg import Pose
from smach import State, UserData

from tiago_controller import TiagoController


class MoveToDiningArea(State):

    def __init__(self, controller: TiagoController):
        super().__init__(outcomes=["success"])
        self.controller: TiagoController = controller
        self.dining_area_x: float = 1.4
        self.dining_area_y: float = -4.5

    def execute(self, userdata: UserData) -> str:
        pose = Pose()
        pose.position.x = self.dining_area_x
        pose.position.y = self.dining_area_y
        pose.orientation.z = 1
        pose.orientation.w = 0
        self.controller.change_pose(pose)
        return "success"
