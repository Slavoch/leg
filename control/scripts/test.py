#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from utils.kinematics import Kinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class Controller:
    def __init__(self) -> None:
        self.kin = Kinematics()
        self.current_angles = np.array([0, 0, 0])

    def start_plan(self):
        self.jt = JointTrajectory()
        self.jt.joint_names = ["Revolute_3", "Revolute_4", "Revolute_5"]

    def set_current_angles(self, angles):
        self.current_angles = angles

    def move_to_position(self, pose, dt):
        angles, state_pred = self.kin.ik(pose, self.current_angles)
        state = self.kin.fk(self.current_angles)[-1]
        print("state, des_state, predicted_state")
        print(state, pose, state_pred)
        print("current_angles, des_angles")
        print(self.current_angles, angles)
        self.jt.points.append(
            JointTrajectoryPoint(
                positions=angles,
                time_from_start=rospy.Duration(dt),
            )
        )


class RosManager:
    def __init__(self) -> None:
        rospy.init_node("listener", anonymous=True)
        self.controller = Controller()
        rospy.Subscriber("/leg/joint_states", JointState, self.listen_joint_state)

    def spin(self):
        rate = rospy.Rate(10)
        pub = rospy.Publisher(
            "leg/joint_trajectory_controller/command", JointTrajectory, queue_size=10
        )
        while not rospy.is_shutdown():
            trajectory = self.plan()
            pub.publish(trajectory)
            rate.sleep()

    def listen_joint_state(self, msg: JointState):
        angles = np.array(msg.position[:-1])
        self.controller.set_current_angles(angles)

    def plan(self):
        self.controller.start_plan()
        self.controller.move_to_position([0.1, 0.3, -0.1], 1)
        trajectory = self.controller.jt
        return trajectory


if __name__ == "__main__":
    ros_manager = RosManager()
    ros_manager.spin()
