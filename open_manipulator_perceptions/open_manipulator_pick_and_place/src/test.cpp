#include <ros/ros.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/SetJointPosition.h>

void sendPose(ros::ServiceClient& ik_client, double x, double y, double z, double path_time = 3.0)
{
  open_manipulator_msgs::SetKinematicsPose pose_srv;
  pose_srv.request.end_effector_name = "gripper";
  pose_srv.request.kinematics_pose.pose.position.x = x;
  pose_srv.request.kinematics_pose.pose.position.y = y;
  pose_srv.request.kinematics_pose.pose.position.z = z;
  pose_srv.request.kinematics_pose.pose.orientation.w = 0.74;
  pose_srv.request.kinematics_pose.pose.orientation.x = 0.0;
  pose_srv.request.kinematics_pose.pose.orientation.y = 0.66;
  pose_srv.request.kinematics_pose.pose.orientation.z = 0.0;
  pose_srv.request.path_time = path_time;

  if (ik_client.call(pose_srv) && pose_srv.response.is_planned)
    ROS_INFO("Moved to (%.3f, %.3f, %.3f)", x, y, z);
  else
    ROS_WARN("Failed to move to (%.3f, %.3f, %.3f)", x, y, z);

  ros::Duration(path_time).sleep();
}

void controlGripper(ros::ServiceClient& gripper_client, double position, double path_time = 2.0)
{
  open_manipulator_msgs::SetJointPosition gripper_srv;
  gripper_srv.request.planning_group = "gripper";
  gripper_srv.request.joint_position.joint_name.push_back("gripper");
  gripper_srv.request.joint_position.position.push_back(position);
  gripper_srv.request.path_time = path_time;

  if (gripper_client.call(gripper_srv) && gripper_srv.response.is_planned)
    ROS_INFO("Gripper moved to position %.3f", position);
  else
    ROS_WARN("Failed to move gripper to position %.3f", position);

  ros::Duration(path_time).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ik_pose_sender");
  ros::NodeHandle nh;

  ros::ServiceClient ik_client = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  ros::ServiceClient gripper_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

  ROS_INFO("Waiting for services...");
  ik_client.waitForExistence();
  gripper_client.waitForExistence();
  ROS_INFO("Services are available.");

  sendPose(ik_client, 0.286,  0.0, 0.204);   
  controlGripper(gripper_client, 0.01);   

  sendPose(ik_client, 0.225,  -0.022, 0.1);      
  controlGripper(gripper_client, 0.01);     

  sendPose(ik_client, 0.240,  0.201, 0);     
  sendPose(ik_client, 0.240,  0.201, 0.2); 

  sendPose(ik_client, 0.162, 0.0, 0.202);  
     
  return 0;
}
