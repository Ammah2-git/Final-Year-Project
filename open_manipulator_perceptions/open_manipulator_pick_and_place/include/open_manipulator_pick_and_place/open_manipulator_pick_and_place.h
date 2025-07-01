#ifndef OPEN_MANIPULATOR_PICK_AND_PLACE_H_
#define OPEN_MANIPULATOR_PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/OpenManipulatorState.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <termios.h> // For kbhit
#include <sys/ioctl.h> // For kbhit
#include <vector> // Required for std::vector
#include <string> // Required for std::string

// Enum for mode states
enum
{
  HOME_POSE = 1,
  DEMO_START = 2,
  DEMO_STOP = 3
};

// Struct for AR Marker data
typedef struct
{
  int id;
  double position[3];
} ArMarker;

// Define number of joints and tool
#define NUM_OF_JOINT_AND_TOOL 5 // joint1, joint2, joint3, joint4, gripper

class OpenManipulatorPickandPlace
{
 private:
  // ROS NodeHandle and Timer
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;
  ros::Timer publish_timer_;

  // ROS Subscribers
  ros::Subscriber open_manipulator_states_sub_;
  ros::Subscriber open_manipulator_joint_states_sub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;
  ros::Subscriber ar_pose_marker_sub_;

  // ROS Service Clients
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient goal_task_space_path_position_only_client_;

  // Robot State Variables
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  geometry_msgs::Pose kinematics_pose_; // Stores full end-effector pose including orientation
  std::vector<std::string> joint_name_;
  bool open_manipulator_is_moving_;

  // Demo Control Variables
  int mode_state_;
  int demo_count_;
  int pick_ar_id_;
  std::vector<ArMarker> ar_marker_pose;
  std::vector<int> processed_ar_ids_;
  
  // NEW: Dynamic home pose to be adjusted based on item position
  std::vector<double> dynamic_home_pose_; 

  // Helper function for keyboard input
  bool kbhit();

  // Callbacks
  void publishCallback(const ros::TimerEvent&);
  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);

  // Robot Command Functions
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setToolControl(std::vector<double> joint_angle);
  bool setTaskSpacePathPositionOnly(double x, double y, double z, double path_time);

  // Demo Logic
  void setModeState(char ch);
  void demoSequence();
  void printText();
  void waitForRobotToStop();

 public:
  OpenManipulatorPickandPlace();
  ~OpenManipulatorPickandPlace();

  // Accessor for kinematics_pose_ (used in main for initialization check)
  geometry_msgs::Pose getKinematicsPose() const { return kinematics_pose_; }

  void initServiceClient();
  void initSubscribe();
};

#endif // OPEN_MANIPULATOR_PICK_AND_PLACE_H_
