from geometry_msgs.msg import Pose
from smach import State, UserData

from tiago_controller import TiagoController


class MoveToKitchen(State):

    def __init__(self, controller: TiagoController):
        super().__init__(outcomes=["success"], input_keys=["order"], output_keys=["order"])
        self.controller: TiagoController = controller
        self.kitchen_area_x: float = 0
        self.kitchen_area_y: float = 0

    def execute(self, userdata: UserData) -> str:
        pose = Pose()
        pose.position.x = self.kitchen_area_x
        pose.position.y = self.kitchen_area_y
        pose.orientation.z = 1
        pose.orientation.w = 0
        self.controller.change_pose(pose)
        return "success"
