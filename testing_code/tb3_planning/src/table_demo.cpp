
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h> //转换函数头文件
int main(int argc, char **argv) {
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
  // MoveIt operates on sets of joints called "planning groups" and stores them
  // in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and
  // "joint model group"
  // are used interchangably.
  // static const std::string PLANNING_GROUP = "arm";
  static const std::string PLANNING_GROUP = "tb3";
  // static const std::string PLANNING_GROUP = "base";
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control
  // and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for
  // improved performance.
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a
  // high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text,
  // cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE,
                           rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to
  // RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s",
                 move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   move_group.setPlannerId("RRTControl");
  move_group.setPlannerId("RRTstar");
  std::string plannerID = move_group.getPlannerId();

  ROS_INFO_STREAM(plannerID);
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the demo");
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  // move_group.setRPYTarget(-pi/2,0,0);

  moveit::core::RobotStatePtr start_state = move_group.getCurrentState();

  std::vector<double> joint_group_positions;
  start_state->copyJointGroupPositions(joint_model_group,
                                       joint_group_positions);

  for (std::vector<double>::iterator it = joint_group_positions.begin();
       it != joint_group_positions.end(); it++) {
    ROS_INFO_STREAM(*(it));
  }

  geometry_msgs::Pose target_pose1;
  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  target_pose1.orientation = quaternion;
  // target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.23;

//   move_group.setPoseTarget(target_pose1);
  move_group.setApproximateJointValueTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group.setPlanningTime(10);
  ROS_INFO_STREAM("----"<<move_group.getPlanningTime()<<"----");
  bool success = (move_group.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.

  //   moveit_msgs::OrientationConstraint ocm;
  //   ocm.link_name = move_group.getEndEffectorLink();
  //   ocm.header.frame_id = "virtual_base_link";
  //   quaternion = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  //   ROS_INFO("****quaternion x=%f y=%f z=%f w=%f****", quaternion.x,
  //   quaternion.y, quaternion.z, quaternion.w);
  // //   ocm.orientation.w = 1.0;
  //   ocm.orientation = quaternion;
  //   ocm.absolute_x_axis_tolerance = 0;
  //   ocm.absolute_y_axis_tolerance = 0;
  //   ocm.absolute_z_axis_tolerance = pi;
  //   ocm.weight = 1.0;

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject table_obj;
  table_obj.header.frame_id = move_group.getPlanningFrame();
  ROS_INFO_STREAM("^^^planning frame is " << table_obj.header.frame_id);

  // The id of the object is used to identify it.
  table_obj.id = "table";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4; // box_x
  primitive.dimensions[1] = 0.4; // box_y
  primitive.dimensions[2] = 0.2; // box_z

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0;
  box_pose.position.z = 0.1;

  table_obj.primitives.push_back(primitive);
  table_obj.primitive_poses.push_back(box_pose);
  table_obj.operation = table_obj.ADD;

  moveit_msgs::CollisionObject obs1;
  obs1.header.frame_id = move_group.getPlanningFrame();
  ROS_INFO_STREAM("^^^planning frame is " << obs1.header.frame_id);
  // The id of the object is used to identify it.
  obs1.id = "obs1";
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive obs1_pri;
  obs1_pri.type = primitive.BOX;
  obs1_pri.dimensions.resize(3);
  obs1_pri.dimensions[0] = 0.1;  // box_x
  obs1_pri.dimensions[1] = 0.1;  // box_y
  obs1_pri.dimensions[2] = 0.05; // box_z

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose obs1_pose;
  obs1_pose.position.x = 0.6;
  obs1_pose.position.y = 0.15;
  obs1_pose.position.z = 0.225;
  obs1.primitives.push_back(obs1_pri);
  obs1.primitive_poses.push_back(obs1_pose);
  obs1.operation = obs1.ADD;

  moveit_msgs::CollisionObject obs2;
  obs2.header.frame_id = move_group.getPlanningFrame();
  ROS_INFO_STREAM("^^^planning frame is " << obs2.header.frame_id);
  // The id of the object is used to identify it.
  obs2.id = "obs2";
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive obs2_pri;
  obs2_pri.type = primitive.BOX;
  obs2_pri.dimensions.resize(3);
  obs2_pri.dimensions[0] = 0.1;  // box_x
  obs2_pri.dimensions[1] = 0.1;  // box_y
  obs2_pri.dimensions[2] = 0.05; // box_z

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose obs2_pose;
  obs2_pose.position.x = 0.6;
  obs2_pose.position.y = -0.15;
  obs2_pose.position.z = 0.225;
  obs2.primitives.push_back(obs2_pri);
  obs2.primitive_poses.push_back(obs2_pose);
  obs2.operation = obs2.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(table_obj);
  //   collision_objects.push_back(obs1);
  //   collision_objects.push_back(obs2);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an table into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add table", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Next: avoid the obstracle");

  moveit_msgs::PositionConstraint pcm;
  pcm.header.frame_id = "virtual_base_link";
  pcm.link_name = move_group.getEndEffectorLink();
  pcm.target_point_offset.x = 1;
  pcm.target_point_offset.y = 1;
  pcm.target_point_offset.z = 0.01;

  shape_msgs::SolidPrimitive bounding_region;
  bounding_region.type = bounding_region.BOX;
  bounding_region.dimensions.resize(3);
  bounding_region.dimensions[0] = 5;
  bounding_region.dimensions[1] = 5;
  bounding_region.dimensions[2] = 0.03;

  geometry_msgs::Pose region_pose;
  obs1_pose.position.x = 0.6;
  obs1_pose.position.y = 0.0;
  obs1_pose.position.z = 0.225;
  pcm.constraint_region.primitives.push_back(bounding_region);
  pcm.constraint_region.primitive_poses.push_back(region_pose);
  pcm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  test_constraints.position_constraints.push_back(pcm);
    // move_group.setPathConstraints(test_constraints);

/*
  // set pose goal
  geometry_msgs::Pose target_pose2;
  //   target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.4;
  target_pose2.position.y = -0.2;
  target_pose2.position.z = 0.23;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pi / 2);
  target_pose2.orientation = quaternion;

  robot_state::RobotState target_state2(*move_group.getCurrentState());
  target_state2.setFromIK(joint_model_group, target_pose1);

  target_state2.copyJointGroupPositions(joint_model_group, joint_group_positions);
  for (std::vector<double>::iterator it = joint_group_positions.begin();
       it != joint_group_positions.end(); it++) {
    ROS_INFO_STREAM(*(it));
  }

  move_group.setStartState(target_state2);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose2);

  // Planning with constraints can be slow because every sample must call an
  // inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the
  // planner has enough time to succeed.
  move_group.setPlanningTime(10.0);
      success = (move_group.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s",
                    success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "start");
  visual_tools.publishAxisLabeled(target_pose2, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE,
                           rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");
*/

// set waypoints
    robot_state::RobotState target_state2(*move_group.getCurrentState());
    target_state2.setFromIK(joint_model_group, target_pose1);
    move_group.setStartState(target_state2);

    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose target_pose2;
    //   target_pose2.orientation.w = 1.0;
    target_pose2.position.x = 0.4;
    target_pose2.position.y = 0;
    target_pose2.position.z = 0.23;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    target_pose2.orientation = quaternion;  
    waypoints.push_back(target_pose2);  //左中


    target_pose2.position.x = 0.4;
    target_pose2.position.y = -0.2;
    target_pose2.position.z = 0.23;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pi / 2);
    target_pose2.orientation = quaternion;  
    waypoints.push_back(target_pose2);  //左下角

    target_pose2.position.x = 0.6;
    target_pose2.position.y = -0.2;
    target_pose2.position.z = 0.23;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pi / 2);
    target_pose2.orientation = quaternion;
    waypoints.push_back(target_pose2);  //下中

    target_pose2.position.x = 0.8;
    target_pose2.position.y = -0.2;
    target_pose2.position.z = 0.23;
    quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pi/2);
    target_pose2.orientation = quaternion;
    waypoints.push_back(target_pose2);  // 右下角

    // target_pose2.position.x = 0.8;
    // target_pose2.position.y = 0;
    // target_pose2.position.z = 0.23;
    // quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pi);
    // target_pose2.orientation = quaternion;
    // waypoints.push_back(target_pose2);  // 右中

    // target_pose2.position.x = 0.8;
    // target_pose2.position.y = 0.2;
    // target_pose2.position.z = 0.23;
    // quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -pi/2);
    // target_pose2.orientation = quaternion;
    // waypoints.push_back(target_pose2);  // 右上角

    // target_pose2.position.x = 0.4;
    // target_pose2.position.y = 0.2;
    // target_pose2.position.z = 0.23;
    // quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    // target_pose2.orientation = quaternion;
    // waypoints.push_back(target_pose2);  // 左上角

    move_group.setMaxVelocityScalingFactor(0.3);


	moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
	const double eef_step = 0.005;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
        while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
 
	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    my_plan.trajectory_ = trajectory;
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);


  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

visual_tools.prompt("next step");

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(table_obj.id);
  object_ids.push_back(obs1.id);
  object_ids.push_back(obs2.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object
   * message */
  visual_tools.prompt("END");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
