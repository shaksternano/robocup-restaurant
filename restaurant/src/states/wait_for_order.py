import rospy
from actionlib import SimpleActionServer
from smach import State, UserData

from restaurant.msg import DeliverOrderAction, DeliverOrderGoal


class WaitForOrder(State):

    def __init__(self):
        super().__init__(outcomes=["success"])
        self.server: SimpleActionServer = SimpleActionServer(
            "deliver_order",
            DeliverOrderAction,
            self.execute_action,
            False,
        )
        self.server.start()
        self.deliver: bool = False

    # noinspection PyUnusedLocal
    def execute_action(self, goal: DeliverOrderGoal):
        self.deliver = True
        self.server.set_succeeded()

    def execute(self, userdata: UserData) -> str:
        while not self.deliver:
            rospy.sleep(1)
        self.deliver = False
        return "success"
