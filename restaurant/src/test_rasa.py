#!/usr/bin/env python

import rospy

from tiago_controller import TiagoController

if __name__ == "__main__":
    rospy.init_node("test_rasa", anonymous=True)
    rospy.loginfo("Testing Rasa service")
    controller = TiagoController()
    order_sentences = {
        "I would like chips": "chips",
        "I would like a burger": "burger",
        "I would like a cola": "cola",
        "Can I have chips": "chips",
        "Can I have a burger": "burger",
        "Can I have a cola": "cola",
        "I want chips": "chips",
        "I want a burger": "burger",
        "I want a cola": "cola",
        "Give me chips": "chips",
        "Give me a burger": "burger",
        "Give me a cola": "cola",
    }
    correct = 0
    for sentence, order in order_sentences.items():
        order_item = controller.get_order_item(sentence)
        rospy.loginfo(f"Testing sentence: {sentence}")
        rospy.loginfo(f"Expected order: {order}")
        rospy.loginfo(f"Actual order: {order_item}")
        if order == order_item:
            rospy.loginfo("Test passed")
            correct += 1
        else:
            rospy.loginfo("Test failed")
    rospy.loginfo(f"{correct}/{len(order_sentences)} tests passed")
