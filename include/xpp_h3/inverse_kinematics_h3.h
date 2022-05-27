#ifndef INVERSE_KINEMATICS_H3_H_
#define INVERSE_KINEMATICS_H3_H_

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_h3/h3leg_inverse_kinematics.h>

namespace xpp {

/**
 * @brief Inverse Kinematics for H3 exoskeleton.
 */
class InverseKinematicsH3 : public InverseKinematics {
public:
  InverseKinematicsH3() = default;
  virtual ~InverseKinematicsH3() = default;

  /**
   * @brief Returns joint angles to reach for a specific foot position.
   * @param pos_B  3D-position of the foot expressed in the base frame (B).
   */
  Joints GetAllJointAngles(const EndeffectorsPos& pos_B) const override;

  /**
   * @brief Number of endeffectors (feet, hands) this implementation expects.
   */
  int GetEECount() const override { return 2; };

private:
  H3legInverseKinematics leg;
};

} /* namespace xpp  */

#endif /* INVERSE_KINEMATICS_H3_H_ */
