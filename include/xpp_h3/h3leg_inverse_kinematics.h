#ifndef H3LEG_INVERSE_KINEMATICS_H_
#define H3LEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>

namespace xpp {

enum H3JointID {H=0, K, A, H3legJointCount};

/**
 * @brief Converts foot position to joint angles.
 */
class H3legInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  H3legInverseKinematics () = default;
  virtual ~H3legInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip (H).
   */
  Vector3d GetJointAngles(const Vector3d& ee_pos_H, KneeBend bend=Forward) const;

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (H, K, A) this value represents.
   */
  void EnforceLimits(double& q, H3JointID joint) const;

};

} /* namespace xpp */

#endif /* H3LEG_INVERSE_KINEMATICS_H_ */
