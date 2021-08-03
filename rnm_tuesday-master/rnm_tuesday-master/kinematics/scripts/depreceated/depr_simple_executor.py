#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from simple_ik import RobotState
from std_msgs.msg import Float64MultiArray, String


class Communicative_Node:
    def __init__(self):
        sim = rospy.get_param('use_sim_time', False)
        self.command_topic = "/joint_position_example_controller/joint_command"
        self.execution_target = 'hardware'
        self.j_states_topic = "/franka_state_controller/joint_states_desired"
        if sim:
            self.command_topic = "/joint_position_example_controller_sim/joint_command"
            self.execution_target = 'simulation'
            self.j_states_topic = "/joint_states"


class Executor(Communicative_Node):
    def __init__(self):
        super(Executor, self).__init__()
        self.publisher = rospy.Publisher(self.command_topic, Float64MultiArray, queue_size=10)
        self.state_publisher = rospy.Publisher('execution_state', String, queue_size=10)
        self.counter = 0
        print("Trajectory executor started for", self.execution_target)

    def execute_plan(self, plan, fr=1000):
        r = rospy.Rate(fr)
        while True:
            if self.counter == plan.shape[1]:
                self.counter = 0
                return
            goal = plan[:, self.counter]
            self.publisher.publish(Float64MultiArray(data=goal))
            self.counter += 1
            r.sleep()

    def perbutrate(self):
        ## DEPRECEATED # Delete if singularity doesn't happen
        ## instead of this you can reinvoke ik_calculate
        print("inside perbutration")
        rob = RobotState()
        print("--for perbutration--")
        j_states = np.array(rospy.wait_for_message(self.j_states_topic, JointState, rospy.Duration(10)).position)
        random_values = np.random.random(7) * (rob.joints_max - rob.joints_min) - rob.joints_min
        print("of range", 0.001)
        random_values *= 0.001
        self.publisher.publish(Float64MultiArray(data=j_states + random_values))
        print("published perbutration")

    def idle(self):
        msg = rospy.wait_for_message('/trajectory_plan', Float64MultiArray)
        # if msg.data[0] == np.inf:
        #    print("perbutration request received, executing")
        #    self.perbutrate()
        #    self.state_publisher.publish("completed")
        #    return
        tr_plan = np.array(msg.data).reshape(7, -1)
        print('Plan received of duration', tr_plan.shape[1] / 1000)
        self.execute_plan(tr_plan)
        self.state_publisher.publish("completed")
        print('Trajectory executed')


if __name__ == "__main__":
    rospy.init_node('trajectory_execution_node')

    executor = Executor()

    while not rospy.is_shutdown():
        executor.idle()
