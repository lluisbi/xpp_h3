#include <ros/ros.h>

#include <xpp_h3/inverse_kinematics_h3.h>
#include <xpp_msgs/topic_names.h>

#include <xpp_vis/urdf_visualizer.h>
#include <xpp_h3/h3_cartesian_joint_converter.h>

#include <xpp_states/endeffector_mappings.h>

using namespace xpp;

int main(int argc, char *argv[]){

  ::ros::init(argc, argv, "h3_urdf_visualizer");
  const std::string joint_desired_h3 = "xpp/joint_h3_des";

  auto ik = std::make_shared<InverseKinematicsH3>();
  H3CartesianJointConverter inv_kin_converter(ik,
					    xpp_msgs::robot_state_desired,
					    joint_desired_h3);

  std::vector<UrdfVisualizer::URDFName> joint_names(6);
  joint_names.at(0) = "left_hip";
  joint_names.at(1) = "left_knee";
  joint_names.at(2) = "left_ankle";
  joint_names.at(3) = "right_hip";
  joint_names.at(4) = "right_knee";
  joint_names.at(5) = "right_ankle";

  std::string urdf = "h3_rviz_urdf_robot_description";
  UrdfVisualizer node_des(urdf, joint_names, "base_link", "world",
			  joint_desired_h3, "h3");

  ::ros::spin();

  return 1;
}
