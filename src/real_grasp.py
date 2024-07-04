#!/usr/bin/env python3

'''
This script is the main loop of the real-world grasping experiments
'''
import rospy
import transforms3d
import numpy as np

import geometry_msgs.msg

from real_gripper_control import GripperControl
from ur_control import UR_robot

def construct_pose(position, angle, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    # pose_under_camera = function(x, y)
    robot_pose = geometry_msgs.msg.Pose()
    robot_pose.position.x = position[0]
    robot_pose.position.y = position[1]
    robot_pose.position.z = position[2]

    ## transform Euler angles into Quternion, angle = (rx, ry, rz)
    quaternion = transforms3d.euler.euler2quat(angle[0], angle[1], angle[2], 'rxyz')

    robot_pose.orientation.w = quaternion[0]
    robot_pose.orientation.x = quaternion[1]
    robot_pose.orientation.y = quaternion[2]
    robot_pose.orientation.z = quaternion[3]

    # print(transforms3d.euler.quat2euler((0, 7.07109031e-01, 7.07104531e-01, 0), 'rxyz'))
    return robot_pose
def moverobot(rotation_matrix, pose_grasp, pose_pre_grasp, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    base2baselink =  np.array([[ -1,  0, 0],
                            [0,  -1,  0],
                            [0, 0, 1]])
    ## Construct robotiq_controller
    gripper = GripperControl()
    # gripper.reset_gripper()
    rospy.sleep(1)

    ur = UR_robot()
    # Grasping angles
    rot_mat = np.matmul(base2baselink, rotation_matrix)
    rot_angle = transforms3d.euler.mat2euler(rot_mat, 'rxyz')


    goal_position = [-pose_grasp[0]/1000,-pose_grasp[1]/1000,pose_grasp[2]/1000-40/1000]
    goal_pose = construct_pose(goal_position[0:3], rot_angle[0:3], frame_id='base_link')

    # Up pose
    up_position = [-pose_pre_grasp[0]/1000,-pose_pre_grasp[1]/1000,pose_pre_grasp[2]/1000-40/1000]
    up_pose = construct_pose(up_position[0:3], rot_angle[0:3], frame_id='base_link')

    # Drop pose
    rot_mat2 = np.array([[ -0.8426062,  0.5372024, -0.0377938],
                            [0.5374385,  0.8432911,  0.0044704],
                            [0.0342727, -0.0165450, -0.9992756]])
    drop_mat = np.matmul(base2baselink, rot_mat2)
    drop_angle = transforms3d.euler.mat2euler(drop_mat, 'rxyz')
    drop_position = [-0.55, 0.41, 0.38]
    drop_pose = construct_pose(drop_position[0:3], drop_angle[0:3], frame_id='base_link')

    ur.go_to_cartesian_pose(up_pose)
    ur.go_to_cartesian_pose(goal_pose)
    rospy.sleep(1)
    gripper.control_gripper(255)
    # gripper.close()
    rospy.sleep(1)
    ur.go_to_cartesian_pose(up_pose)
    ur.go_to_cartesian_pose(drop_pose)
    rospy.sleep(1)
    gripper.control_gripper(0)
    # gripper.open()
    rospy.sleep(1)
