#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from geometry_msgs.msg import Quaternion

def send_pose(ik_client, x, y, z, path_time=2.0):
    req = SetKinematicsPose._request_class()

    req.end_effector_name = "gripper"
    req.kinematics_pose.pose.position.x = x
    req.kinematics_pose.pose.position.y = y
    req.kinematics_pose.pose.position.z = z

    req.kinematics_pose.pose.orientation.w = 1.0
    req.kinematics_pose.pose.orientation.x = 0.0
    req.kinematics_pose.pose.orientation.y = 0.0
    req.kinematics_pose.pose.orientation.z = 0.0

    req.path_time = path_time

    try:
        resp = ik_client(req)
        if resp.is_planned:
            rospy.loginfo(f"Moved to ({x:.3f}, {y:.3f}, {z:.3f})")
        else:
            rospy.logwarn(f"Failed to move to ({x:.3f}, {y:.3f}, {z:.3f})")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    rospy.sleep(path_time)

def control_gripper(gripper_client, position, path_time=1.0):
    req = SetJointPosition._request_class()

    req.planning_group = "gripper"
    req.joint_position.joint_name.append("gripper")
    req.joint_position.position.append(position)
    req.path_time = path_time

    try:
        resp = gripper_client(req)
        if resp.is_planned:
            rospy.loginfo(f"Gripper moved to position {position:.3f}")
        else:
            rospy.logwarn(f"Failed to move gripper to position {position:.3f}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    rospy.sleep(path_time)

if __name__ == "__main__":
    rospy.init_node("test_ik_pose_sender")

    rospy.loginfo("Waiting for services...")
    ik_client = rospy.ServiceProxy("goal_task_space_path", SetKinematicsPose)
    gripper_client = rospy.ServiceProxy("goal_tool_control", SetJointPosition)
    ik_client.wait_for_service()
    gripper_client.wait_for_service()
    rospy.loginfo("Services are available.")

    send_pose(ik_client, 0.286,  0.0, 0.204)
    control_gripper(gripper_client, 0.01)

    send_pose(ik_client, 0.240,  0.201, 0.0)
    control_gripper(gripper_client, 0.005)

    send_pose(ik_client, 0.240,  0.201, 0.1)
    send_pose(ik_client, 0.240,  0.201, 0.2)

    send_pose(ik_client, -0.162, 0.0, 0.202)
