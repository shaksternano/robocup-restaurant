import rospy

from smach import State, UserData


class Wait(State):

    def __init__(self, seconds: float):
        super().__init__(outcomes=["success"])
        self.seconds: float = seconds

    def execute(self, userdata: UserData) -> str:
        rospy.sleep(self.seconds)
        return "success"
