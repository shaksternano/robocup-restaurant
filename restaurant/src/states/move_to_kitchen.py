from geometry_msgs.msg import Pose
from smach import State, UserData

from context import Context
from tiago_controller import TiagoController


class MoveToKitchen(State):

    def __init__(self, controller: TiagoController, context: Context):
        super().__init__(outcomes=["success"])
        self.controller: TiagoController = controller
        self.context: Context = context

    def execute(self, userdata: UserData) -> str:
        pose = Pose()
        pose.position.x = self.context.kitchen_area_x
        pose.position.y = self.context.kitchen_area_y
        pose.orientation.z = 1
        pose.orientation.w = 0
        self.controller.change_pose(pose)
        return "success"
