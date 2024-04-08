#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from control.scripts.utils.foKin import kinematics


def listen_joint_state(msg: JointState):
    angles = msg.position[:-1]
    points = kinematics(angles)
    print(angles)


if __name__ == "__main__":
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/leg/joint_states", JointState, listen_joint_state)
    plt.show(block=True)
    rospy.spin()
