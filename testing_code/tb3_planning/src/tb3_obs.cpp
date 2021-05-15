
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model/joint_model.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb3_wiping");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::float_t pi = 3.1415926;

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  // static const std::string PLANNING_GROUP = "arm";
  static const std::string PLANNING_GROUP = "tb3";
  // static const std::string PLANNING_GROUP = "base";
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;



  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  unsigned int var_cnt = joint_model_group->getVariableCount();

  std::vector<const robot_model::JointModel*> joint_model_vector = joint_model_group->getActiveJointModels();

  for(std::vector<const robot_model::JointModel*>::iterator it  = joint_model_vector.begin(); it != joint_model_vector.end(); it++)
  {
    ROS_INFO_STREAM((*it)->getName());
  }


  ROS_INFO_STREAM("FUUUUUUUUUUUUUCKKKKKKKKK   "<<var_cnt);

  ROS_INFO("FUUUUUUUUUUUUUCKKKKKKKKK");
  // move_group.setPlannerId("RRTConnect");
  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  move_group.setPlannerId("RRTstar");
  std::string plannerID = move_group.getPlannerId();
  
  ROS_INFO_STREAM(plannerID);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  // move_group.setRPYTarget(-pi/2,0,0);


  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for(std::vector<double>::iterator it  = joint_group_positions.begin(); it != joint_group_positions.end(); it++)
    {
      ROS_INFO_STREAM(*(it));
    }
  



  geometry_msgs::Pose target_pose1;
  geometry_msgs::Quaternion quaternion;
  // quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,0,pi/4);
  // target_pose1.orientation = quaternion;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.6;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.3;
  // move_group.setPoseTarget(target_pose1);

  moveit::core::RobotState goalstate1(*current_state);
  goalstate1.setFromIK(joint_model_group, target_pose1);
  
  ROS_INFO_STREAM("After IK");
  goalstate1.copyJointGroupPositions(joint_model_group, joint_group_positions);
  for(std::vector<double>::iterator it  = joint_group_positions.begin(); it != joint_group_positions.end(); it++)
    {
      ROS_INFO_STREAM(*(it));
    }

  // move_group.setPoseTarget(target_pose1);
   move_group.setApproximateJointValueTarget(target_pose1);


  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.setPlanningTime(10.0);
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  ros::shutdown();
  return 0;
}
