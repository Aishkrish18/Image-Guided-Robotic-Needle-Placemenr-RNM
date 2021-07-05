#!/usr/bin/env python3
# Cenk


import rospy
from std_msgs.msg import Float64MultiArray

class Executor():
    def __init__(self, command_topic):
        self.publisher = rospy.Publisher(command_topic, Float64MultiArray, queue_size=10)
        self.counter = 0

    def send_step_command(self, plan):
        if self.counter == plan.shape[1]:
            return 'Done'
        goal = plan[:, self.counter]
        msg = Float64MultiArray()
        msg.data = goal
        self.publisher.publish(msg)
        self.counter += 1
