import rospy

from smach import State, UserData


class Wait(State):

    def __init__(self):
        super().__init__(
            outcomes=["success"],
        )

    def execute(self, userdata: UserData) -> str:
        rospy.sleep(10)
        return "success"
