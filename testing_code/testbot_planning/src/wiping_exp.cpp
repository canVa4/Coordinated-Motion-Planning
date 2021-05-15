
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model/joint_model.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <robot_model/joint_model.h>

#include<tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wiping_exp");
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
  static const std::string PLANNING_GROUP = "base_arm";
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
  const robot_model::JointModel* virtual_r_model= joint_model_group->getJointModel("virtual_rotation");
  double dis_fact = virtual_r_model->getDistanceFactor();
  ROS_INFO_STREAM("Distance factor:"<<dis_fact);
  // double new_dis_fact = 0.3;
  // virtual_r_model->setDistanceFactor(new_dis_fact); 

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
  moveit_visual_tools::MoveItVisualTools visual_tools("virtual_base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.5;
  visual_tools.publishText(text_pose, "wiping exp", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("wiping exp", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("wiping exp", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  move_group.setPlannerId("RRTstar");
//   move_group.setPlannerId("RRT");
  std::string plannerID = move_group.getPlannerId();
  
  ROS_INFO_STREAM(plannerID);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Adding a wall");
  //****************************
  //*******Adding Walls*********
  //****************************
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "wall";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 2.4;
  primitive.dimensions[2] = 1.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  box_pose.position.x = 1;
  box_pose.position.y = 1;
  box_pose.position.z = 0.7;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  std::vector<moveit_msgs::ObjectColor> obj_colors;
  moveit_msgs::ObjectColor wall_color;
  wall_color.id = "wall";
  wall_color.color.r = 225.0/255;
  wall_color.color.g = 85.0/255;
  wall_color.color.b = 0;
  wall_color.color.a = 0.95;
  obj_colors.push_back(wall_color);
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
   visual_tools.prompt("Press 'next' Adding obs");

  //****************************
  //*******Adding obs*********
  //****************************

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Adding obstacles", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  moveit_msgs::CollisionObject obs1;
  obs1.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  obs1.id = "obs1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive obs1_prim;
  obs1_prim.type = obs1_prim.BOX;
  obs1_prim.dimensions.resize(3);
  obs1_prim.dimensions[0] = 0.3;
  obs1_prim.dimensions[1] = 0.3;
  obs1_prim.dimensions[2] = 0.3;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose obs1_pose;
  // box_pose.orientation.w = 1.0;
  obs1_pose.position.x = 0.8;
  obs1_pose.position.y = 1;
  obs1_pose.position.z = 0.15;

  obs1.primitives.push_back(obs1_prim);
  obs1.primitive_poses.push_back(obs1_pose);
  obs1.operation = obs1.ADD;

  // std::vector<moveit_msgs::CollisionObject> obs_s;
  collision_objects.push_back(obs1);
  moveit_msgs::ObjectColor obs1_color;
  obs1_color.id = "obs1";
  obs1_color.color.r = 170.0/255;
  obs1_color.color.g = 170.0/255;
  obs1_color.color.b = 0;
  obs1_color.color.a = 0.8;
  obj_colors.push_back(obs1_color);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Adding obstacles");
  // planning_scene_interface.addCollisionObjects(obs_s);
  planning_scene_interface.applyCollisionObjects(collision_objects, obj_colors);
  visual_tools.prompt("Press 'next' Start to plan");

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
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,pi/2,0);
  target_pose1.orientation = quaternion;
  // target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.8;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.7;
  // move_group.setPoseTarget(target_pose1);

  moveit::core::RobotState goalstate1(*current_state);
  goalstate1.setFromIK(joint_model_group, target_pose1);
  
  ROS_INFO_STREAM("After IK");
  goalstate1.copyJointGroupPositions(joint_model_group, joint_group_positions);
  for(std::vector<double>::iterator it  = joint_group_positions.begin(); it != joint_group_positions.end(); it++)
    {
      ROS_INFO_STREAM(*(it));
    }

  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(10);
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  bool success; 
  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("No Obstacle Demo", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("No Obstacle Demo", "Visualizing plan 1 as trajectory line");

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to excute the plan");

  move_group.move();
  
  visual_tools.prompt("Press 'next' to plan");

  // move_group.setStartState(goalstate1);
  move_group.setStartStateToCurrentState();

  geometry_msgs::Pose target_pose2;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,pi/2,0);
  target_pose2.orientation = quaternion;
  // target_pose1.orientation.w = 1.0;
  target_pose2.position.x = 0.8;
  target_pose2.position.y = 1.5;
  target_pose2.position.z = 0.7;
  move_group.setPoseTarget(target_pose2);
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.trigger();

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link8";
  ocm.header.frame_id = "virtual_base_link";
  ocm.orientation = quaternion;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = 0.2;
  ocm.weight = 1.0;

  // moveit_msgs::PositionConstraint pcm;
  // pcm.link_name = "panda_link8";
  // pcm.header.frame_id = "virtual_base_link";
  // pcm.orientation = quaternion

  // moveit_msgs::BoundingVolume constraints_bv;
  // shape_msgs::SolidPrimitive constraints_pm;
  // constraints_pm.type = constraints_pm.BOX;
  // constraints_pm.dimensions.resize(3);
  // constraints_pm.dimensions[0] = 3;
  // constraints_pm.dimensions[1] = 3;
  // constraints_pm.dimensions[2] = 0.05;

  // // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose obs1_pose;
  // // box_pose.orientation.w = 1.0;
  // obs1_pose.position.x = 0.4;
  // obs1_pose.position.y = 1;
  // obs1_pose.position.z = 0.2;
  // bv.primitives.push_back()
  // pcm.target_point_offset.x = 0;
  // pcm.target_point_offset.y = 0;
  // pcm.target_point_offset.z = 0;
  // pcm.weight = 1.0;
  move_group.setPlanningTime(20);
  moveit_msgs::Constraints robot_constraints;
  robot_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(robot_constraints);

  ROS_INFO_NAMED("No Obstacle Demo", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("No Obstacle Demo", "Visualizing plan 1 as trajectory line");


  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to end the demo");
  ros::shutdown();


  return 0;
}
