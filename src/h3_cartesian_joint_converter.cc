#include <xpp_h3/h3_cartesian_joint_converter.h>

#include <ros/node_handle.h>

#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>

namespace xpp {

H3CartesianJointConverter::H3CartesianJointConverter (const InverseKinematicsH3::Ptr& ik,
                                                      const std::string& cart_topic,
                                                      const std::string& joint_topic)
{
  inverse_kinematics_ = ik;

  ::ros::NodeHandle n;
  cart_state_sub_ = n.subscribe(cart_topic, 1, &H3CartesianJointConverter::StateCallback, this);
  ROS_DEBUG("Subscribed to: %s", cart_state_sub_.getTopic().c_str());

  joint_state_pub_  = n.advertise<xpp_msgs::RobotStateJoint>(joint_topic, 1);
  ROS_DEBUG("Publishing to: %s", joint_state_pub_.getTopic().c_str());
}

void
H3CartesianJointConverter::StateCallback (const xpp_msgs::RobotStateCartesian& cart_msg)
{
  auto cart = Convert::ToXpp(cart_msg);

  // transform feet from world -> base frame
  Eigen::Matrix3d B_R_W = cart.base_.ang.q.normalized().toRotationMatrix().inverse();
  EndeffectorsPos ee_B(cart.ee_motion_.GetEECount());
  for (auto ee : ee_B.GetEEsOrdered())
    ee_B.at(ee) = B_R_W * (cart.ee_motion_.at(ee).p_ - cart.base_.lin.p_);

  Eigen::VectorXd q =  inverse_kinematics_->GetAllJointAngles(ee_B).ToVec();

  xpp_msgs::RobotStateJoint joint_msg;
  joint_msg.base            = cart_msg.base;
  joint_msg.ee_contact      = cart_msg.ee_contact;
  joint_msg.time_from_start = cart_msg.time_from_start;
  joint_msg.joint_state.position = std::vector<double>(q.data(), q.data()+q.size());
  // Attention: Not filling joint velocities or torques

  joint_state_pub_.publish(joint_msg);
}

} /* namespace xpp */
