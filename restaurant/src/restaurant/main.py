#!/usr/bin/env python

import rospy

from state_machine import Tables

if __name__ == "__main__":
    rospy.init_node("visit_tables", anonymous=True)
    state_machine = Tables()
    state_machine.execute()
