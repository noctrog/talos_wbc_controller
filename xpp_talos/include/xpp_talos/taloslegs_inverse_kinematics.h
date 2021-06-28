#ifndef TALOSLEGS_INVERSE_KINEMATICS_H_
#define TALOSLEGS_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

namespace xpp {

  enum TalosJointID {
    LL1=0, LL2, LL3, LL4, LL5, LL6,                 // Left leg
    LR1, LR2, LR3, LR4, LR5, LR6,                   // Right leg
    T1, T2,                                         // Torso
    AL1, AL2, AL3, AL4, AL5, AL6, AL7, GL,          // Arm left
    AR1, AR2, AR3, AR4, AR5, AR6, AR7, GR,          // Arm right
    H1, H2, TalosJointCount };                      // Head

  enum TalosLeg {LEFTLEG = 0, RIGHTLEG, LEGCOUNT};

  /**
   * @brief Converts a talos left foot position to joint angles
   */
  class TalosLegsInverseKinematics
  {
  public:
    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;

    /**
     * @brief Default c'tor
     */
    TalosLegsInverseKinematics() = default;
    virtual ~TalosLegsInverseKinematics() = default;
    
    /**
     * @brief Returns the joint angles to reach a foot position and orientation (6 values)
     * @param Leg on which perform inverse kinematics (LEFTLEG, RIGHTLEG)
     * @param ee_pos_H Foot position xyz w.r.t the base link (TODO: comprobar)
     * @param ee_or_H Foot orientation w.r.t. the base link
     */
    Eigen::VectorXd 
    GetJointAngles(const TalosLeg leg,
		   const Vector3d& ee_pos_H,
		   const Matrix3d& orientation = Matrix3d::Identity()) const;
  private:

  };

}

#endif /* end of include guard: TALOSLEGS_INVERSE_KINEMATICS_H_IN3VHQ2E */
