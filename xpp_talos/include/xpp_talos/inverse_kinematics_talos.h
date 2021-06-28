#ifndef INVERSE_KINEMATICS_TALOS_H_
#define INVERSE_KINEMATICS_TALOS_H_

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 30

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_talos/taloslegs_inverse_kinematics.h>

namespace xpp {

  using EndeffectorsRot = Endeffectors<Eigen::Matrix3d>;

  /**
   * @brief Inverse Kinematics for the Talos legs.
   */
  class InverseKinematicsTalos : public InverseKinematics {
  public:
    using Ptr = std::shared_ptr<InverseKinematicsTalos>;

    InverseKinematicsTalos();
    virtual ~InverseKinematicsTalos() = default;

    /**
     * @brief Returns joint angles to reach for a specific foot position.
     * @param pos_B  3D-position of the foot expressed in the base frame (B).
     */
    Joints GetAllJointAngles(const EndeffectorsPos& pos_B) const override;

    /**
     * @brief Return joint angles to reach for a specific foot position.
     * @param pos_B  3D-position of the foot expressed in the base frame (B)
     * @param rot_B  Orientation of the end effector with respect to the base frame (B)
     */
    Joints GetAllJointAngles(const EndeffectorsPos& pos_B,
			     const EndeffectorsRot& rot_B) const;

    /**
     * @brief Return joint velocities.
     * @param vel_B  3D-velocity of the foot expressed in the base frame (B)
     * @param pos_j  Current positions for every joint.
     */
    Joints GetAllJointVelocities(const EndeffectorsVel &vel_B,
				 const Joints &pos_j) const;

    /**
     * @brief Return joint velocities.
     * @param acc_B  3D-acceleration of the foot expressed in the base frame (B)
     * @param pos_j  Current positions for every joint.
     * @param vel_j  Current velocities for every joint.
     */
    Joints GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
				    const Joints &pos_j,
				    const Joints &vel_j) const;

    /**
     * @brief Return joint velocities.
     * @param vel_B  3D-velocity of the foot expressed in the base frame (B)
     * @param q  Current positions for every joint.
     */
    Joints GetAllJointVelocities(const EndeffectorsVel &vel_B,
				 const Eigen::VectorXd &q) const;

    /**
     * @brief Return joint velocities.
     * @param acc_B  3D-acceleration of the foot expressed in the base frame (B)
     * @param q  Current positions for every joint.
     * @param qd  Current velocities for every joint.
     */
    Joints GetAllJointAccelerations(const EndeffectorsAcc &acc_B,
				    const Eigen::VectorXd &q,
				    const Eigen::VectorXd &qd) const;

    /**
     * @brief      Return the center of mass position 
     * @param      Current robot state.
     * @return     Center of mass position
     */
    Eigen::Vector3d GetCenterOfMassPosition(const Eigen::VectorXd& q) const;
    
    /**
     * @brief      Return the center of mass velocity
     * @param      Current robot state.
     * @return     Center of mass velocity
     */
    Eigen::Vector3d GetCenterOfMassVelocity(const Eigen::VectorXd& q,
					    const Eigen::VectorXd& qd) const;

    /**
     * @brief      Return the center of mass position and velocity.
     */
    void GetCenterOfMassPositionAndVelocity(const Eigen::VectorXd& q,
					    const Eigen::VectorXd& qd,
					    Eigen::Vector3d& com_pos,
					    Eigen::Vector3d& com_vel) const;
    /**
     * @brief      Return the center of mass position, velocity and acceleration.
     */
    void GetCenterOfMassPositionVelocityAcceleration(const Eigen::VectorXd& q,
						     const Eigen::VectorXd& qd,
						     const Eigen::VectorXd& qdd,
						     Eigen::Vector3d& com_pos,
						     Eigen::Vector3d& com_vel,
						     Eigen::Vector3d& com_acc) const;
    /**
     * @brief Number of endeffectors (2 feet).
     */
    int GetEECount() const override { return 2; };

  private:

    /**
     * @brief Returns the Jacobian of the frame specified.
     * 
     * Before calling this method, pinocchio::computeJointJacobians
     * and pinocchio::updateFramePlacements must be called first.
     * 
     */
    Eigen::MatrixXd
    GetFrameJacobian(int id) const;

    /**
     * @brief Returns the Jacobian time derivative of the frame specified.
     * 
     * Before calling this method, pinocchio::computeJointJacobiansTimeVariation
     * and pinocchio::updateFramePlacements must be called first.
     * 
     */
    Eigen::MatrixXd
    GetFrameJacobianTimeDerivative(int id) const;
    
    TalosLegsInverseKinematics legs;

    typedef pinocchio::Model Model;
    typedef pinocchio::Data Data;
    typedef std::shared_ptr<Model> ModelPtr;
    typedef std::shared_ptr<Data> DataPtr;
    ModelPtr talos_model_;
    DataPtr talos_data_;

    int left_sole_id, right_sole_id;
  };

} /* namespace xpp  */

#endif /* end of include guard: INVERSE_KINEMATICS_TALOS_H_ */
