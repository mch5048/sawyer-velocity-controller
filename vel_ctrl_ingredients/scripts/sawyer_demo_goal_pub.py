#!/usr/bin/env python
# ROS Imports
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose


class GoalPublisher(object):
    def __init__(self):

        self.cur_pos = np.zeros(3)
        self.goal_pub = rospy.Publisher("/teacher/ik_vel/", Pose, queue_size=3)
        self.ee_pose_sub = rospy.Subscriber(
            "/open_manipulator/end_effector_pose", Pose, self.ee_pose_cb
        )
        self.is_set_target = False
        self.is_ee_pose_cb = False
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0
        while self.is_ee_pose_cb is False:
            pass
        print("Home position X: 0.138 Y: 0.0 Z: 0.239")
        print("Init position X: 0.290 Y: 0.0 Z: 0.203")
        comment = (
            "Select Mode "
            + "(0: safe init, 1 : safe home, "
            + "2 : init position, 3: home position 4: user define)"
        )
        self.mode = input(comment)
        rospy.set_param("bool_demo_run", True)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.goal_publish()
            if np.mean(np.abs(self.cur_pos - self.target)) < 0.001:
                rospy.set_param("bool_demo_run", False)
                break
            rate.sleep()

    def goal_publish(self):

        if self.mode == 0:
            if self.is_set_target is False:
                self.control_start_time = (
                    rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
                )
                self.control_duration = 2.0
                self.target_x = 0.290
                self.target_y = 0.0
                self.target_z = 0.203
                self.target = np.array([self.target_x, self.target_y, self.target_z])
                self.target = self.operation_limit_check(self.target)
                self.is_set_target = True
            self.now = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
            self.goal_x = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_x,
                self.target_x,
                0.0,
                0.0,
            )
            self.goal_y = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_y,
                self.target_y,
                0.0,
                0.0,
            )
            self.goal_z = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_z,
                self.target_z,
                0.0,
                0.0,
            )

        elif self.mode == 1:
            if self.is_set_target is False:
                self.control_start_time = (
                    rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
                )
                self.control_duration = 2.0
                self.target_x = 0.138
                self.target_y = 0.0
                self.target_z = 0.239
                self.target = np.array([self.target_x, self.target_y, self.target_z])
                self.target = self.operation_limit_check(self.target)
                self.is_set_target = True
            self.now = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
            self.goal_x = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_x,
                self.target_x,
                0.0,
                0.0,
            )
            self.goal_y = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_y,
                self.target_y,
                0.0,
                0.0,
            )
            self.goal_z = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_z,
                self.target_z,
                0.0,
                0.0,
            )

        elif self.mode == 2:
            self.goal_x = 0.290
            self.goal_y = 0.0
            self.goal_z = 0.203
            self.target = np.array([self.goal_x, self.goal_y, self.goal_z])
            self.target = self.operation_limit_check(self.target)

        elif self.mode == 3:
            self.goal_x = 0.138
            self.goal_y = 0.0
            self.goal_z = 0.239
            self.target = np.array([self.goal_x, self.goal_y, self.goal_z])
            self.target = self.operation_limit_check(self.target)
            self.control_start_time = (
                rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
            )

        elif self.mode == 4:
            if self.is_set_target is False:
                comment = (
                    "Enter goal position x, y, z, control duration: "
                    + "ex) 0.2 0.0 0.2 2.0\n"
                )
                if sys.version_info[0] == 3:
                    self.target_x, self.target_y, self.target_z, self.control_duration = [
                        float(goal) for goal in input(comment).split()
                    ]
                elif sys.version_info[0] == 2:
                    self.target_x, self.target_y, self.target_z, self.control_duration = [
                        float(goal) for goal in raw_input(comment).split()
                    ]
                self.target = np.array([self.target_x, self.target_y, self.target_z])
                self.target = self.operation_limit_check(self.target)
                self.control_start_time = (
                    rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
                )
                self.is_set_target = True
            self.now = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 10 ** -9
            self.goal_x = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_x,
                self.target[0],
                0.0,
                0.0,
            )
            self.goal_y = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_y,
                self.target[1],
                0.0,
                0.0,
            )
            self.goal_z = self.cubic(
                self.now,
                self.control_start_time,
                self.control_start_time + self.control_duration,
                self.init_z,
                self.target[2],
                0.0,
                0.0,
            )

        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        goal_pose.position.z = self.goal_z
        # Trick : control duration is published instead of orientation.x value
        goal_pose.orientation.w = self.control_duration
        self.goal_pub.publish(goal_pose)

    def ee_pose_cb(self, ee_pose):
        if self.is_set_target is False:
            self.init_x = ee_pose.position.x
            self.init_y = ee_pose.position.y
            self.init_z = ee_pose.position.z
            self.is_ee_pose_cb = True

        self.cur_pos[0] = ee_pose.position.x
        self.cur_pos[1] = ee_pose.position.y
        self.cur_pos[2] = ee_pose.position.z

    def cubic(self, time, time_0, time_f, x_0, x_f, x_dot_0, x_dot_f):
        x_t = x_0

        if time < time_0:
            x_t = x_0

        elif time > time_f:
            x_t = x_f
        else:
            elapsed_time = time - time_0
            total_time = time_f - time_0
            total_time2 = total_time * total_time
            total_time3 = total_time2 * total_time
            total_x = x_f - x_0

            x_t = (
                x_0
                + x_dot_0 * elapsed_time
                + (
                    3 * total_x / total_time2
                    - 2 * x_dot_0 / total_time
                    - x_dot_f / total_time
                )
                * elapsed_time
                * elapsed_time
                + (-2 * total_x / total_time3 + (x_dot_0 + x_dot_f) / total_time2)
                * elapsed_time
                * elapsed_time
                * elapsed_time
            )

        return x_t

    def operation_limit_check(self, target):
        target = self.operation_box_limit_check(target)
        target = self.operation_sphere_limit_check(target)
        return target

    def operation_box_limit_check(self, target):
        lower_limit = [0.0, -0.15, 0.04]  # x, y, z
        upper_limit = [0.4, 0.15, 0.3]
        for i in range(3):
            if target[i] < lower_limit[i]:
                target[i] = lower_limit[i]
                print("Target is out of box boundary!")
            if target[i] > upper_limit[i]:
                target[i] = upper_limit[i]
                print("Target is out of box boundary!")
        return target

    def operation_sphere_limit_check(self, target):
        max_op_distance = 0.4
        if np.linalg.norm(target) > max_op_distance:
            target = target * max_op_distance / np.linalg.norm(target)
            print("Target is out of sphere boundary!")
        return target


def main():
    rospy.init_node("goal_publisher")

    try:
        GoalPublisher()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()