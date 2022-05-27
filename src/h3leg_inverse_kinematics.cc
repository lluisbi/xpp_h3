#include <xpp_h3/h3leg_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>

namespace xpp {

H3legInverseKinematics::Vector3d
H3legInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B, KneeBend bend) const
{

  double l1 = 0.1;
  double l3 = 0.3915;
  double l4 = 0.3915;
  double l5 = 0.1605;

  double x = ee_pos_B[X] - l5;
  double z = ee_pos_B[Z] + 0.0385;

  double D, q1, q2, q3;

  // IK 1
  D = (pow(x,2) + pow(z,2) - pow(l3,2) - pow(l4,2)) / ( 2 * l3 * l4 );

  q2 = atan2(sqrt(1-pow(D,2)),D);

  q1 = - (M_PI/2 - abs(atan2(z,x) + atan2(l4*sin(q2),l3 + l4*cos(q2))));

  q3 = (M_PI/2 + atan2(z - (-l3*cos(abs(q1))), x - l3*sin(abs(q1))));

  if(ee_pos_B[x] >= 0.02){

    if(isnan(q1)){q1=0.0;}
    if(isnan(q2)){q2=0.0;}
    if(isnan(q3)){q3=0.0;}

    EnforceLimits(q1,H);
    EnforceLimits(q2,K);
    EnforceLimits(q3,A);

    return Vector3d(q1,q2,q3);
  }

  // IK 2
  double old_value = abs(ee_pos_B[X]);
  double old_min = 0.0;
  double old_max = 0.25;
  double new_min = 0;
  double new_max = 0.2;

  double new_value = ( (old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min;
  z = z + new_value;
  x = ee_pos_B[X]- abs(sqrt(pow(l5,2)-pow(new_value,2)));

  D = (pow(x,2) + pow(z,2) - pow(l3,2) - pow(l4,2)) / ( 2 * l3 * l4 );

  q2 = atan2(sqrt(1-pow(D,2)),D);

  q1 = - (M_PI/2 - abs(atan2(z,x) + atan2(l4*sin(q2),l3 + l4*cos(q2))));

  double z1 = -l3 * cos(abs(q1));
  double x1 = -l3 * sin(abs(q1));
    
  double v = sqrt(pow(x-x1,2)+pow(z-z1,2));

  double alpha = acos((pow(l4,2)+pow(l5,2)-pow(v,2))/(2*l4*l5));

  q3 = abs(alpha) - M_PI/2;

  if(isnan(q1)){q1=0.0;}
  if(isnan(q2)){q2=0.0;}
  if(isnan(q3)){q3=0.0;}

  EnforceLimits(q1,H);
  EnforceLimits(q2,K);
  EnforceLimits(q3,A);

  return Vector3d(q1,q2,q3);
}

void
H3legInverseKinematics::EnforceLimits (double& val, H3JointID joint) const
{
  // totally exaggerated joint angle limits
  const static double h_min = -105;
  const static double h_max =  93;

  const static double k_min = -5;
  const static double k_max =  105;

  const static double a_min = -30;
  const static double a_max =  30;

  // reduced joint angles for optimization
  static const std::map<H3JointID, double> max_range {
    {H, h_max/180.0*M_PI},
    {K, k_max/180.0*M_PI},
    {A, a_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<H3JointID, double> min_range {
    {H, h_min/180.0*M_PI},
    {K, k_min/180.0*M_PI},
    {A, a_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;

}

} /* namespace xpp */
