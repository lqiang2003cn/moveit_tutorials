/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <sstream>
#include <tf/transform_datatypes.h>

double box_x = 4;
double box_y = -2.7;
double box_z = 0.8393;

void pickMineVersion(moveit::planning_interface::MoveGroupInterface &move_group){
    move_group.setStartState(*move_group.getCurrentState());

//    arm_link_7:
//    position: 3.5836; -2.722; 0.31194
//    orientation: -0.0053417; 0.0032312; -0.69324; 0.72068

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "odom";
    goal_pose.pose.position.x = box_x;
    goal_pose.pose.position.y = box_y;
    goal_pose.pose.position.z = 1;
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
//    goal_pose.pose.orientation.x = -0.0053417;
//    goal_pose.pose.orientation.y =  0.0032312;
//    goal_pose.pose.orientation.z = -0.69324;
//    goal_pose.pose.orientation.w = 0.72068;
    move_group.setPoseTarget(goal_pose);

    ROS_INFO_STREAM(
            "Planning to move " << move_group.getEndEffectorLink() <<
                                " to a target pose expressed in " << move_group.getPlanningFrame()
    );

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // exe
    ros::Time start = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode e = move_group.move();
    if (!bool(e))
        throw std::runtime_error("Error executing plan");
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

}

void openGripper(trajectory_msgs::JointTrajectory &posture) {
    posture.joint_names.resize(2);
    posture.joint_names[0] = "gripper_left_left_finger_joint";
    posture.joint_names[1] = "gripper_left_right_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture) {
    posture.joint_names.resize(2);
    posture.joint_names[0] = "gripper_left_left_finger_joint";
    posture.joint_names[1] = "gripper_left_right_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group) {
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].allowed_touch_objects.emplace_back("gripper_left");
    grasps[0].grasp_pose.header.frame_id = "map";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = box_x;
    grasps[0].grasp_pose.pose.position.y = box_y;
    grasps[0].grasp_pose.pose.position.z = box_z;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "map";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "map";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);
    closedGripper(grasps[0].grasp_posture);
    move_group.setSupportSurfaceName("table0");
    move_group.pick("cube0", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &group) {
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in
    // verbose mode." This is a known issue and we are working on fixing it. |br|
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
    // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    collision_objects[0].id = "table0";
    collision_objects[0].header.frame_id = "map";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 0.8;
    collision_objects[0].primitives[0].dimensions[2] = 0.03;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 4.2;
    collision_objects[0].primitive_poses[0].position.y = -3;
    collision_objects[0].primitive_poses[0].position.z = 0.8;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, 1.57);
    myQuaternion = myQuaternion.normalize();
    collision_objects[0].primitive_poses[0].orientation.x = myQuaternion.x();
    collision_objects[0].primitive_poses[0].orientation.y = myQuaternion.y();
    collision_objects[0].primitive_poses[0].orientation.z = myQuaternion.z();
    collision_objects[0].primitive_poses[0].orientation.w = myQuaternion.w();
    collision_objects[0].operation = collision_objects[0].ADD;

//    box_pose.pose.position.x = 4.0
//    box_pose.pose.position.y = -2.7
//    box_pose.pose.position.z = 0.8393
//    box_pose.pose.orientation.x = 0
//    box_pose.pose.orientation.y = 0
//    box_pose.pose.orientation.z = 0
//    box_pose.pose.orientation.w = 1.0
//    box_name = "box"
//    scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))

    collision_objects[1].id = "cube0";
    collision_objects[1].header.frame_id = "map";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.05;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;
    collision_objects[1].primitives[0].dimensions[2] = 0.05;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = box_x;
    collision_objects[1].primitive_poses[0].position.y = box_y;
    collision_objects[1].primitive_poses[0].position.z = box_z;
    collision_objects[1].operation = collision_objects[1].ADD;


//  collision_objects[2].header.frame_id = "panda_link0";
//  collision_objects[2].id = "object";
//
//  /* Define the primitive and its dimensions. */
//  collision_objects[2].primitives.resize(1);
//  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
//  collision_objects[2].primitives[0].dimensions.resize(3);
//  collision_objects[2].primitives[0].dimensions[0] = 0.02;
//  collision_objects[2].primitives[0].dimensions[1] = 0.02;
//  collision_objects[2].primitives[0].dimensions[2] = 0.2;
//
//  /* Define the pose of the object. */
//  collision_objects[2].primitive_poses.resize(1);
//  collision_objects[2].primitive_poses[0].position.x = 0.5;
//  collision_objects[2].primitive_poses[0].position.y = 0;
//  collision_objects[2].primitive_poses[0].position.z = 0.5;
//  // END_SUB_TUTORIAL
//
//  collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void move_to_pose(const std::string& arm_name, double x, double y, double z, double rx, double ry, double rz) {
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal_pose.pose.position.z = z;
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rx, ry, rz);

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::vector<std::string> torso_arm_joint_names;
    //select group of joints
    std::string moveit_group = "arm_" + arm_name + "_torso";
    moveit::planning_interface::MoveGroupInterface group_arm_torso(moveit_group);
    //choose your preferred planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(goal_pose);

    ROS_INFO_STREAM("Planning to move " <<
                                        group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                                        group_arm_torso.getPlanningFrame());

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group_arm_torso.setPlanningTime(5.0);
    bool success = bool(group_arm_torso.plan(my_plan));

    if (!success)
        throw std::runtime_error("No plan found");
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    ros::Time start = ros::Time::now();
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
    if (!bool(e))
        throw std::runtime_error("Error executing plan");
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    spinner.stop();
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm_left_torso");
    group.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(1.0).sleep();
    pickMineVersion(group);

//    pick(group);
//
//    ros::WallDuration(1.0).sleep();
//
//    place(group);
//
//    ros::waitForShutdown();
    return 0;
}

