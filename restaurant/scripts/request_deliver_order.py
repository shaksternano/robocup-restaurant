#!/usr/bin/env python

import actionlib
import rospy

from restaurant.msg import DeliverOrderAction, DeliverOrderGoal

if __name__ == "__main__":
    rospy.init_node("deliver_order_client")
    client = actionlib.SimpleActionClient("deliver_order", DeliverOrderAction)
    client.wait_for_server()
    goal = DeliverOrderGoal()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5))
