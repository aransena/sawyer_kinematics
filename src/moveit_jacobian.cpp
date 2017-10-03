//
// Created by aransena on 02/10/17.
//

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sawyer_jacobian");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "right_arm";
    move_group_interface::MoveGroup move_group(PLANNING_GROUP);

    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;

    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;

    ros::Rate rate(50);
    while (n.ok()) {
        current_state = move_group.getCurrentState();
        jacobian = current_state->getJacobian(joint_model_group, reference_point_position);
        ROS_INFO_STREAM("Jacobian: \n" << jacobian);
        rate.sleep();
    }

//    const std::vector<std::string> &joint_names = joint_model_group->getActiveJointModelNames();
//
//    for(std::size_t i = 0; i < joint_names.size(); ++i)
//    {
//        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);//joint_values[i]);
//    }


}

