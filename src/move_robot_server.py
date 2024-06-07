#!/usr/bin/env python3
from move_robot.srv import MoveToPose,MoveToPoseResponse #注意是功能包名.srv
import rospy
import numpy as np
from grasp import moverobot

def handle_move_to_pose(req):
    rotation_matrix,pose_g, pose_up = np.array(req.rotation_matrix).reshape(3,3), np.array(req.grasp_pose), np.array(req.up_pose)
    moverobot(rotation_matrix,pose_g, pose_up)
    return MoveToPoseResponse(True)

if __name__ == "__main__":
    server = rospy.Service('move_to_pose_service',MoveToPose, handle_move_to_pose)
    rospy.loginfo("Service server ready to take pose goals.")
    rospy.spin()
