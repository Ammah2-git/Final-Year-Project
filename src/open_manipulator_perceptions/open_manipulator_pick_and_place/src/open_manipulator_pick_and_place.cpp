/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_pick_and_place/open_manipulator_pick_and_place.h"
#include <cstdio> // For printf
#include <unistd.h> // For usleep
#include <algorithm> // For std::find

OpenManipulatorPickandPlace::OpenManipulatorPickandPlace()
: node_handle_(""),
  priv_node_handle_("~"),
  mode_state_(0),
  demo_count_(0),
  pick_ar_id_(-1) // Initialize to -1 to indicate no target picked yet
{
  present_joint_angle_.resize(NUM_OF_JOINT_AND_TOOL, 0.0);
  present_kinematic_position_.resize(3, 0.0);

  joint_name_.push_back("joint1");
  joint_name_.push_back("joint2");
  joint_name_.push_back("joint3");
  joint_name_.push_back("joint4");

  // NEW: Initialize the dynamic home pose with default values
  dynamic_home_pose_ = {0.0, -1.05, 0.35, 1.5};

  initServiceClient();
  initSubscribe();

  // Initialize kinematics_pose_ to prevent using uninitialized values
  kinematics_pose_.orientation.w = 1.0;
  kinematics_pose_.orientation.x = 0.0;
  kinematics_pose_.orientation.y = 0.0;
  kinematics_pose_.orientation.z = 0.0;

  publish_timer_ = node_handle_.createTimer(ros::Duration(0.100), &OpenManipulatorPickandPlace::publishCallback, this);
}

OpenManipulatorPickandPlace::~OpenManipulatorPickandPlace()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void OpenManipulatorPickandPlace::initServiceClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
}

void OpenManipulatorPickandPlace::initSubscribe()
{
  open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &OpenManipulatorPickandPlace::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorPickandPlace::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("gripper/kinematics_pose", 10, &OpenManipulatorPickandPlace::kinematicsPoseCallback, this);
  ar_pose_marker_sub_ = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorPickandPlace::arPoseMarkerCallback, this);
}

bool OpenManipulatorPickandPlace::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  printf("Commanding joint space path to: [");
  for (size_t i = 0; i < joint_angle.size(); ++i) {
      printf("%.3f%s", joint_angle[i], (i == joint_angle.size() - 1 ? "" : ", "));
  }
  printf("] with path_time: %.1f\n", path_time);

  if (goal_joint_space_path_client_.call(srv))
  {
    if (srv.response.is_planned) {
      printf("Joint space path planned successfully.\n");
      return true;
    } else {
      printf("Joint space path failed to plan. Is IK possible for these angles?\n");
      return false;
    }
  }
  printf("Failed to call joint space path service.\n");
  return false;
}

bool OpenManipulatorPickandPlace::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = 0.5; // Gripper moves relatively fast

  printf("Commanding gripper to angle: %.3f...\n", joint_angle.at(0));
  if (goal_tool_control_client_.call(srv))
  {
    if (srv.response.is_planned) {
      printf("Gripper control planned successfully.\n");
      return true;
    } else {
      printf("Gripper control failed to plan.\n");
      return false;
    }
  }
  printf("Failed to call gripper control service.\n");
  return false;
}

bool OpenManipulatorPickandPlace::setTaskSpacePathPositionOnly(double x, double y, double z, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = x;
  srv.request.kinematics_pose.pose.position.y = y;
  srv.request.kinematics_pose.pose.position.z = z;

  srv.request.kinematics_pose.pose.orientation = kinematics_pose_.orientation;

  srv.request.path_time = path_time;

  printf("Commanding task space path to (X:%.3f, Y:%.3f, Z:%.3f) with path_time: %.1f\n", x, y, z, path_time);
  printf("Using Orientation W:%.3f, X:%.3f, Y:%.3f, Z:%.3f\n",
         kinematics_pose_.orientation.w,
         kinematics_pose_.orientation.x,
         kinematics_pose_.orientation.y,
         kinematics_pose_.orientation.z);

  if (goal_task_space_path_position_only_client_.call(srv))
  {
    if (srv.response.is_planned) {
      printf("Task space path planned successfully.\n");
      return true;
    } else {
      printf("Task space path failed to plan. IK failure or out of reach.\n");
      printf("Requested Pose: X=%.3f, Y=%.3f, Z=%.3f, Q.W=%.3f, Q.X=%.3f, Q.Y=%.3f, Q.Z=%.3f\n",
             x, y, z,
             kinematics_pose_.orientation.w,
             kinematics_pose_.orientation.x,
             kinematics_pose_.orientation.y,
             kinematics_pose_.orientation.z);
      return false;
    }
  }
  printf("Failed to call task space path service (position_only).\n");
  return false;
}

void OpenManipulatorPickandPlace::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if (msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;
}

void OpenManipulatorPickandPlace::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for (size_t i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("gripper"))  temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorPickandPlace::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  present_kinematic_position_.at(0) = msg->pose.position.x;
  present_kinematic_position_.at(1) = msg->pose.position.y;
  present_kinematic_position_.at(2) = msg->pose.position.z;

  kinematics_pose_ = msg->pose;
}

void OpenManipulatorPickandPlace::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  ar_marker_pose.clear();
  for (size_t i = 0; i < msg->markers.size(); i ++)
  {
    ArMarker temp;
    temp.id = msg->markers.at(i).id;
    temp.position[0] = msg->markers.at(i).pose.pose.position.x;
    temp.position[1] = msg->markers.at(i).pose.pose.position.y;
    temp.position[2] = msg->markers.at(i).pose.pose.position.z;

    ar_marker_pose.push_back(temp);
  }
}

void OpenManipulatorPickandPlace::waitForRobotToStop()
{
    ros::Rate loop_rate(100);
    printf("Waiting for robot to stop...\n");
    while (ros::ok() && open_manipulator_is_moving_) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    printf("Robot has stopped.\n");
}

void OpenManipulatorPickandPlace::publishCallback(const ros::TimerEvent&)
{
  printText();
  if (kbhit()) setModeState(std::getchar());

  if (mode_state_ == HOME_POSE)
  {
    if (!setJointSpacePath(joint_name_, dynamic_home_pose_, 2.0)) {
        printf("Failed to command home pose. Retrying...\n");
        return;
    }
    waitForRobotToStop();

    std::vector<double> gripper_value;
    gripper_value.push_back(0.010);
    if (!setToolControl(gripper_value)) {
        printf("Failed to command gripper open. Retrying...\n");
        return;
    }
    waitForRobotToStop();

    mode_state_ = 0;
    demo_count_ = 0;
    processed_ar_ids_.clear(); 
  }
  else if (mode_state_ == DEMO_START)
  {
    if (!open_manipulator_is_moving_)
    {
      demoSequence();
    }
  }
  else if (mode_state_ == DEMO_STOP)
  {
    // Do nothing when stopped
  }
}

struct DropOffLocation {
    std::vector<double> above;
    std::vector<double> drop;
};

// MODIFIED: Drop off locations now correspond to Marker ID 1, 2, 3
// dropOffs[0] is for Marker 1, dropOffs[1] for Marker 2, etc.
const DropOffLocation dropOffs[3] = {
    // Item 1 (AR Marker 1)
    { {1.867, -0.874, 0.387, 1.493}, {1.878, -0.913, 1.004, 0.989} },
    // Item 2 (AR Marker 2)
    { {3.088, -0.270, 0.268, 0.891}, {3.098, -0.179, 0.732, 0.319} },
    // Item 3 (AR Marker 3)
    { {-1.7, -1.321, 0.726, 1.265}, {-1.710, -1.342, 1.268, 0.880} },
};

const double GripperRelease = 0.010;
const double GripperGrip = -0.002;

void OpenManipulatorPickandPlace::setModeState(char ch)
{
  if (ch == '1')
  {
    mode_state_ = HOME_POSE;
    dynamic_home_pose_ = {0.0, -1.05, 0.35, 1.5};
  }
  else if (ch == '2')
  {
    mode_state_ = DEMO_START;
    demo_count_ = 0;
    pick_ar_id_ = -1;
    processed_ar_ids_.clear();
    dynamic_home_pose_ = {0.0, -1.05, 0.35, 1.5};
    printf("Starting Pick and Place Demo. Press '3' to stop.\n");
  }
  else if (ch == '3')
  {
    mode_state_ = DEMO_STOP;
    printf("Pick and Place Demo Stopped by user.\n");
  }
}

void OpenManipulatorPickandPlace::demoSequence()
{
  std::vector<double> joint_angle;
  std::vector<double> gripper_value;

  double path_time_long = 2.0;
  double path_time_short = 1.0;

  const double SAFE_Z_APPROACH = 0.13;
  const double INTERMEDIATE_Z = 0.05;
  const double ACTUAL_PICK_Z = 0.0;

  // MODIFIED: Define marker ID range for clarity
  const int MIN_AR_ID = 1;
  const int MAX_AR_ID = 3;
  const int TOTAL_EXPECTED_ITEMS = 3; 

  switch (demo_count_)
  {
    case 0: // Go to Initial Home Pose
      printf("DEMO STEP 0: Moving to initial home pose...\n");
      if (setJointSpacePath(joint_name_, dynamic_home_pose_, path_time_long)) {
          waitForRobotToStop();
          demo_count_++;
      } else {
          printf("Command failed. Retrying home pose.\n");
      }
      break;

    case 1: // Open the gripper
      printf("DEMO STEP 1: Opening gripper...\n");
      gripper_value.push_back(GripperRelease);
      if (setToolControl(gripper_value)) {
          waitForRobotToStop();
          demo_count_++;
      } else {
          printf("Command failed. Retrying gripper open.\n");
      }
      break;

    case 2: // Find next AR marker and set the dynamic home pose
    {
      printf("DEMO STEP 2: Searching for next AR marker...\n");
      
      int lowest_ar_id_found = -1;

      // MODIFIED: Loop through the new marker ID range (1, 2, 3)
      for (int current_possible_id = MIN_AR_ID; current_possible_id <= MAX_AR_ID; ++current_possible_id)
      {
          bool is_processed = (std::find(processed_ar_ids_.begin(), processed_ar_ids_.end(), current_possible_id) != processed_ar_ids_.end());
          if (!is_processed)
          {
              for (size_t i = 0; i < ar_marker_pose.size(); ++i)
              {
                  if (ar_marker_pose.at(i).id == current_possible_id)
                  {
                      lowest_ar_id_found = current_possible_id;
                      break;
                  }
              }
          }
          if (lowest_ar_id_found != -1) break;
      }

      if (lowest_ar_id_found == -1)
      {
          if (processed_ar_ids_.size() == TOTAL_EXPECTED_ITEMS) {
              printf("All known AR markers have been processed. Stopping demo.\n");
              mode_state_ = DEMO_STOP;
              demo_count_ = 0;
              pick_ar_id_ = -1;
          } else if (ar_marker_pose.empty()){
              printf("No AR markers detected. Waiting for detection.\n");
          } else {
              printf("No *new* AR markers detected that haven't been processed. Waiting...\n");
          }
          return; // Stay in this state and wait
      }
      
      pick_ar_id_ = lowest_ar_id_found;

      double item_x = 0.0;
      bool marker_found = false;
      for (size_t i = 0; i < ar_marker_pose.size(); ++i)
      {
          if (ar_marker_pose.at(i).id == pick_ar_id_) {
              item_x = ar_marker_pose.at(i).position[0];
              marker_found = true;
              break;
          }
      }
      
      if(marker_found)
      {
        printf("... Adjusting home pose based on item X-position: %.3f\n", item_x);
        if (item_x >= 0.240) {
            dynamic_home_pose_.at(3) = 1.15; // Set Joint 4 for far items
            printf("... Set FAR home pose, Joint 4 = 1.75 rad\n");
        } else if (item_x <= 0.180) {
            dynamic_home_pose_.at(3) = 1.75;  // Set Joint 4 for near items
            printf("... Set NEAR home pose, Joint 4 = 1.20 rad\n");
        } else {
            dynamic_home_pose_.at(3) = 1.5;  // Set Joint 4 for medium items
            printf("... Set MEDIUM home pose, Joint 4 = 1.50 rad\n");
        }
        demo_count_++; // Move to next state to execute the move
      }
      else
      {
          printf("Targeted AR marker ID %d disappeared before planning. Re-searching.\n", pick_ar_id_);
          pick_ar_id_ = -1; // Stay in state 2 to re-search
      }
    }
    break;

    case 3: 
      printf("DEMO STEP 3: Moving to optimized home pose for picking...\n");
      if (setJointSpacePath(joint_name_, dynamic_home_pose_, path_time_long)) {
          waitForRobotToStop();
          demo_count_++; // Move to the approach state
      } else {
          printf("Command failed to move to optimized home pose. Retrying search.\n");
          demo_count_ = 2; // Go back to search
          pick_ar_id_ = -1;
      }
      break;

    case 4: 
    {
        printf("DEMO STEP 4: Approaching target object (ID %d)...\n", pick_ar_id_);
        
        double item_x = 0.0, item_y = 0.0;
        bool marker_data_valid = false;
        for (size_t i = 0; i < ar_marker_pose.size(); ++i)
        {
            if (ar_marker_pose.at(i).id == pick_ar_id_) {
                item_x = ar_marker_pose.at(i).position[0];
                item_y = ar_marker_pose.at(i).position[1];
                marker_data_valid = true;
                break;
            }
        }
        
        if (marker_data_valid) {
            printf("Targeting Marker ID %d at X:%.3f, Y:%.3f. Moving to safe approach height.\n", pick_ar_id_, item_x, item_y);
            if (setTaskSpacePathPositionOnly(item_x, item_y, SAFE_Z_APPROACH, path_time_long))
            {
              waitForRobotToStop();
              demo_count_++;
            }
            else
            {
              printf("IK FAILED for initial approach from optimized pose. Retrying search.\n");
              demo_count_ = 2;
              pick_ar_id_ = -1;
            }
        } else {
            printf("Targeted AR marker ID %d disappeared before approach. Re-searching.\n", pick_ar_id_);
            pick_ar_id_ = -1;
            demo_count_ = 2;
        }
    }
    break;

    case 5:
      printf("DEMO STEP 5: Descending to intermediate Z (%.3f)...\n", INTERMEDIATE_Z);
      if (setTaskSpacePathPositionOnly(present_kinematic_position_.at(0), present_kinematic_position_.at(1), INTERMEDIATE_Z, path_time_short)) {
        waitForRobotToStop();
        demo_count_++;
      } else {
        printf("IK FAILED for intermediate Z. Returning to detection.\n");
        demo_count_ = 2;
        pick_ar_id_ = -1;
      }
      break;

    case 6:
      printf("DEMO STEP 6: Descending to actual pick Z (%.3f)....\n", ACTUAL_PICK_Z);
      if (setTaskSpacePathPositionOnly(present_kinematic_position_.at(0), present_kinematic_position_.at(1), ACTUAL_PICK_Z, path_time_short)) {
        waitForRobotToStop();
        demo_count_++;
      } else {
        printf("IK FAILED for final pick Z. Returning to detection.\n");
        demo_count_ = 2;
        pick_ar_id_ = -1;
      }
      break;

    case 7:
      printf("DEMO STEP 7: Closing gripper to pick (ID %d)...\n", pick_ar_id_);
      gripper_value.push_back(GripperGrip);
      if (setToolControl(gripper_value)) {
          waitForRobotToStop();
          demo_count_++;
      } else {
          printf("Command failed. Retrying gripper close.\n");
      }
      break;

    case 8:
      printf("DEMO STEP 8: Ascending from pick Z to intermediate Z (%.3f)...\n", INTERMEDIATE_Z);
      if (setTaskSpacePathPositionOnly(present_kinematic_position_.at(0), present_kinematic_position_.at(1), INTERMEDIATE_Z, path_time_short)) {
        waitForRobotToStop();
        demo_count_++;
      } else {
        printf("IK FAILED for ascending to intermediate Z. Returning to detection.\n");
        demo_count_ = 2;
        pick_ar_id_ = -1;
      }
      break;

    case 9:
      printf("DEMO STEP 9: Ascending to safe Z (%.3f)...\n", SAFE_Z_APPROACH);
      if (setTaskSpacePathPositionOnly(present_kinematic_position_.at(0), present_kinematic_position_.at(1), SAFE_Z_APPROACH, path_time_short)) {
        waitForRobotToStop();
        demo_count_++;
      } else {
        printf("IK FAILED for ascending to safe Z. Returning to detection.\n");
        demo_count_ = 2;
        pick_ar_id_ = -1;
      }
      break;
   
    case 10:
     // MODIFIED: Check for valid marker ID range
     if (pick_ar_id_ < MIN_AR_ID || pick_ar_id_ > MAX_AR_ID) {
         printf("Invalid pick_ar_id_ %d for drop-off. Returning to search.\n", pick_ar_id_);
         demo_count_ = 2;
         return;
     }
     printf("DEMO STEP 10: Moving above drop-off for item ID %d\n", pick_ar_id_);
     // MODIFIED: Map marker ID (1,2,3) to array index (0,1,2)
     joint_angle = dropOffs[pick_ar_id_ - 1].above;
     if (setJointSpacePath(joint_name_, joint_angle, path_time_long)) {
         waitForRobotToStop();
         demo_count_++;
     } else {
        printf("Command failed. Retrying move above drop-off.\n");
     }
     break;

    case 11:
     if (pick_ar_id_ < MIN_AR_ID || pick_ar_id_ > MAX_AR_ID) {
         printf("Invalid pick_ar_id_ %d for drop-off. Returning to search.\n", pick_ar_id_);
         demo_count_ = 2;
         return;
     }
     printf("DEMO STEP 11: Descending to drop for item ID %d\n", pick_ar_id_);
     // MODIFIED: Map marker ID (1,2,3) to array index (0,1,2)
     joint_angle = dropOffs[pick_ar_id_ - 1].drop;
     if (setJointSpacePath(joint_name_, joint_angle, path_time_long)) {
         waitForRobotToStop();
         demo_count_++;
     } else {
         printf("Command failed. Retrying descend to drop.\n");
    }
    break;

    case 12:
     printf("DEMO STEP 12: Opening gripper to release item ID %d\n", pick_ar_id_);
     gripper_value.clear();
     gripper_value.push_back(GripperRelease);
     if (setToolControl(gripper_value)) {
         ros::Duration(0.5).sleep(); 
         waitForRobotToStop();
         demo_count_++;
     } else {
         printf("Command failed. Retrying gripper release.\n");
     }
     break;

    case 13:
     if (pick_ar_id_ < MIN_AR_ID || pick_ar_id_ > MAX_AR_ID) {
         printf("Invalid pick_ar_id_ %d for drop-off. Returning to search.\n", pick_ar_id_);
         demo_count_ = 2;
         return;
     }
     printf("DEMO STEP 13: Moving above drop-off again for item ID %d\n", pick_ar_id_);
     // MODIFIED: Map marker ID (1,2,3) to array index (0,1,2)
     joint_angle = dropOffs[pick_ar_id_ - 1].above;
     if (setJointSpacePath(joint_name_, joint_angle, path_time_long)) {
         waitForRobotToStop();
         demo_count_++;
     } else {
         printf("Command failed. Retrying move above drop-off after release.\n");
     }
     break;

    case 14:
     printf("DEMO STEP 14: Returning to home pose to search for next item...\n");
     joint_angle = dynamic_home_pose_;
     if (setJointSpacePath(joint_name_, joint_angle, path_time_long)) {
        waitForRobotToStop();
        processed_ar_ids_.push_back(pick_ar_id_);
        pick_ar_id_ = -1;
        // Go back to search for the next item
        demo_count_ = 2; 
     } else {
        printf("Command failed. Retrying final return to home pose.\n");
     }
     break;

    default:
      printf("ERROR: Unknown demo_count_ state: %d. Resetting to search state.\n", demo_count_);
      demo_count_ = 2;
      pick_ar_id_ = -1;
      break;
  }
}


void OpenManipulatorPickandPlace::printText()
{
  system("clear");

  printf("\n");
  printf("-----------------------------\n");
  printf("Pick and Place demonstration!\n");
  printf("-----------------------------\n");

  printf("1 : Home pose\n");
  printf("2 : Pick and Place demo. start\n");
  printf("3 : Pick and Place demo. Stop\n");

  printf("-----------------------------\n");

  if (mode_state_ == DEMO_START)
  {
    printf("DEMO STATE: Running - Step %d\n", demo_count_);
    switch(demo_count_)
    {
      case 0:
        printf("Moving to initial home pose.\n");
        break;
      case 1:
        printf("Opening gripper.\n");
        break;
      case 2:
        printf("Searching for next AR marker to pick...\n");
        if (ar_marker_pose.empty()) printf("   (No AR marker currently detected)\n");
        break;
      case 3:
        printf("Moving to optimized home pose for picking.\n");
        break;
      case 4:
        printf("Approaching target object (ID %d).\n", pick_ar_id_);
        break;
      case 5:
        printf("Descending to intermediate Z.\n");
        break;
      case 6:
        printf("Descending to pick Z.\n");
        break;
      case 7:
        printf("Closing gripper (picking item ID %d).\n", pick_ar_id_);
        break;
      case 8:
        printf("Ascending from pick Z to intermediate Z.\n");
        break;
      case 9:
        printf("Ascending to safe Z.\n");
        break;
      case 10:
        printf("Moving to drop-off pose for item ID %d.\n", pick_ar_id_);
        break;
      case 11:
        printf("Descending to drop-off location for item ID %d.\n", pick_ar_id_);
        break;
      case 12:
        printf("Opening gripper to release item ID %d.\n", pick_ar_id_);
        break;
      case 13:
        printf("Moving above drop-off after release for item ID %d.\n", pick_ar_id_);
        break;
      case 14:
        printf("Returning to home pose to search for next item.\n");
        break;
      default:
        printf("Unknown state!\n");
        break;
    }
  }
  else if (mode_state_ == DEMO_STOP)
  {
    printf("The demo has finished or was stopped.\n");
    // MODIFIED: Update the completion message for the new ID range
    if (processed_ar_ids_.size() == 3)
    {
        printf("--- ALL ITEMS (ID 1-3) HAVE BEEN SUCCESSFULLY SORTED! ---\n");
    }
  }
  else
  {
    printf("Press '2' to start pick and place demo.\n");
  }

  printf("-----------------------------\n");
  printf("Robot Moving State: %s\n", open_manipulator_is_moving_ ? "MOVING" : "STOPPED");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         present_joint_angle_.at(0),
         present_joint_angle_.at(1),
         present_joint_angle_.at(2),
         present_joint_angle_.at(3));
  printf("Present Tool Position: %.3lf\n", present_joint_angle_.at(4));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         present_kinematic_position_.at(0),
         present_kinematic_position_.at(1),
         present_kinematic_position_.at(2));
  printf("Present Kinematics Orientation W: %.3lf X: %.3lf Y: %.3lf Z: %.3lf\n",
         kinematics_pose_.orientation.w,
         kinematics_pose_.orientation.x,
         kinematics_pose_.orientation.y,
         kinematics_pose_.orientation.z);
  printf("-----------------------------\n");
  
  if (ar_marker_pose.size()) printf("Currently detected AR markers:\n");
  else printf("No AR markers currently detected.\n");
  for (size_t i = 0; i < ar_marker_pose.size(); ++i)
  {
    printf("ID: %d --> X: %.3lf\tY: %.3lf\tZ: %.3lf\n",
           ar_marker_pose.at(i).id,
           ar_marker_pose.at(i).position[0],
           ar_marker_pose.at(i).position[1],
           ar_marker_pose.at(i).position[2]);
  }
  
  printf("Sorted AR IDs: [");
  if (processed_ar_ids_.empty()) {
      printf("None");
  } else {
      for (size_t i = 0; i < processed_ar_ids_.size(); ++i) {
          printf("%d%s", processed_ar_ids_[i], (i == processed_ar_ids_.size() - 1 ? "" : ", "));
      }
  }
  printf("]\n");
}

bool OpenManipulatorPickandPlace::kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_pick_and_place");
  ros::NodeHandle node_handle("");

  OpenManipulatorPickandPlace open_manipulator_pick_and_place;

  ROS_INFO("Waiting for initial robot state and kinematics_pose to be populated...");
  ros::Rate init_rate(10);
  int num_kinematics_pose_messages = 0;
  while (ros::ok() && num_kinematics_pose_messages < 5)
  {
      ros::spinOnce();
      // Check if the orientation quaternion is not zero
      if (open_manipulator_pick_and_place.getKinematicsPose().orientation.w != 0.0 ||
          open_manipulator_pick_and_place.getKinematicsPose().orientation.x != 0.0 ||
          open_manipulator_pick_and_place.getKinematicsPose().orientation.y != 0.0 ||
          open_manipulator_pick_and_place.getKinematicsPose().orientation.z != 0.0)
      {
          num_kinematics_pose_messages++;
      }
      init_rate.sleep();
  }
  if (!ros::ok()) return 0;
  ROS_INFO("Initial robot state and kinematics_pose received. Starting main loop.");

  ros::spin(); // Use ros::spin() to let ROS handle the callbacks and timers

  return 0;
}
