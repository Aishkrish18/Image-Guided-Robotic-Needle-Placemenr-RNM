#!/usr/bin/env python3

import rospy
import numpy as np
from simple_fk import RobotState
from simple_ik import InverseKinematics
from sensor_msgs.msg import JointState
from simple_trajectory_executer import Executor


def position(t):
    return np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5])


def velocity(t):
    return np.array([0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4])


def acceleration(t):
    return np.array([0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3])


def jerk(t):
    return np.array([0, 0, 0, 6, 24 * t, 60 * t ** 2])


def quintic_mat(t):
    return np.array([position(t), velocity(t), acceleration(t), jerk(t)])


def quintic_coeff_mat(t0, tf):
    return np.vstack([quintic_mat(t0)[:-1], quintic_mat(tf)[:-1]])


def find_quintic_coeffs(q_i, q_t, duration):
    b = [q_i, 0, 0, q_t, 0, 0]
    return np.linalg.solve(quintic_coeff_mat(0, duration), b)


def get_max_values_from_traj(coeffs, dur):
    max_vel_point = velocity(dur / 2)
    x_cric = 0.21132487  # root of the jacob scaled to the dur
    # acceleration is a bit risky since the root changes with res, you can give a safety boundary
    max_acc_point = acceleration(dur * x_cric)
    max_jerk_point = jerk(0)
    max_vels = max_vel_point @ coeffs.T
    max_accs = max_acc_point @ coeffs.T
    max_jerks = max_jerk_point @ coeffs.T
    return max_vels, max_accs, max_jerks


def trajectory_creator(current, goal, dur):
    res = 1000  # 1000 Hz
    pos_eqs = np.array([position(t) for t in np.arange(0, dur, 1 / res)])
    coeff_neu = np.array([find_quintic_coeffs(current[j], goal[j], dur) for j in range(7)])
    v, a, j = get_max_values_from_traj(coeff_neu, dur)
    if RobotState().check_limits(v, a, j):
        # print(dur,'is enough')
        paths_neu = np.array([pos_eqs @ c for c in coeff_neu])
        return paths_neu
    # else:
    #    print(dur,'not enough')


def trajectory_planner(current, goal):
    plan = None
    dur = 0.1
    while plan is None:
        dur += 0.1
        plan = trajectory_creator(current, goal, dur)
    return plan


if __name__ == "__main__":
    rospy.init_node('trajectory_planner_node')

    initial_pose = np.array(rospy.wait_for_message('/joint_states', JointState, rospy.Duration(10)).position)
    ik = InverseKinematics('/our_goal')
    goal_joint_states = ik.calculate_joint_parameters()

    execut = Executor("/joint_position_example_controller_sim/joint_command")

    plan = trajectory_planner(initial_pose, goal_joint_states)

    r = rospy.Rate(1000)
    reverse = False
    while not rospy.is_shutdown():
        msg = execut.send_step_command(plan)
        if msg == 'Done':
            if reverse:
                execut.counter = 0
                print('Trajectory executed,reversing')
                plan = plan[:, ::-1]
            else:
                exit('Trajectory completed')
        r.sleep()
