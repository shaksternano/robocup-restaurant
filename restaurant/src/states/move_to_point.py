import math

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from smach import State, UserData

from tiago_controller import TiagoController


class MoveToPoint(State):

    def __init__(self, controller: TiagoController):
        super().__init__(
            outcomes=["success", "failure"],
            input_keys=["x", "y"],
        )
        self.controller: TiagoController = controller

    def execute(self, userdata: UserData) -> str:
        x: float = userdata["x"]
        y: float = userdata["y"]
        current_x, current_y, _ = self.controller.get_current_pose()
        angle = math.atan2(y - current_y, x - current_x)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = math.sin(angle / 2)
        pose.orientation.w = math.cos(angle / 2)
        result_state = self.controller.change_pose(pose, prevent_crashes=True)
        if result_state == GoalStatus.ABORTED:
            return "failure"
        else:
            return "success"
