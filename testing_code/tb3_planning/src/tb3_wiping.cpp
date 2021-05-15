
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
  target_pose1.position.x = 0.8;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.8;
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


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = move_group.getEndEffectorLink();
  ocm.header.frame_id = "base_footprint";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // moveit_msgs::PositionConstraint pcm;
  // pcm.header.frame_id = "base_footprint";
  // pcm.link_name = move_group.getEndEffectorLink();
  // pcm.target_point_offset.x = 0.01;
  // pcm.target_point_offset.y = 0.01;
  // pcm.target_point_offset.z = 0.01;

  // shape_msgs::SolidPrimitive bounding_region;
  // bounding_region.type = bounding_region.BOX;
  // bounding_region.dimensions.resize(3);
  // bounding_region.dimensions[0] = 0.01;
  // bounding_region.dimensions[1] = 5;
  // bounding_region.dimensions[2] = 5;
  // pcm.constraint_region.primitives.push_back(bounding_region);
  // pcm.constraint_region.primitive_poses.push_back(target_pose1);
  // pcm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  // test_constraints.position_constraints.push_back(pcm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState target_state2(*move_group.getCurrentState());

  geometry_msgs::Pose target_pose2;
//   target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.2;
  target_pose2.position.y = 0;
  target_pose2.position.z = 0.35;
  target_state2.setFromIK(joint_model_group, target_pose1);
  move_group.setStartState(target_state2);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose2);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "start");
  visual_tools.publishAxisLabeled(target_pose2, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.02;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.25;
  box_pose.position.y = 0;
  box_pose.position.z = 0.3;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Next: avoid the obstracle");

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(target_state2);
  move_group.setPoseTarget(target_pose2);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("END");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
