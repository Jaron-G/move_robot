#!/usr/bin/env python3
import rospy
from move_robot.srv import MoveToPose,MoveToPoseResponse #注意是功能包名.srv

def move_robot_to_pose(x, y, z):
    rospy.wait_for_service('move_to_pose')
    try:
        move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
        resp1 = move_to_pose(x, y, z)
        return resp1.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node("move_robot_client")
    result = move_robot_to_pose()
    if result:
        rospy.loginfo("Robot moved successfully!")
    else:
        rospy.loginfo("Failed to move robot.")
    

