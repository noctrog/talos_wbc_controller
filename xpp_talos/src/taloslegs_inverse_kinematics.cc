#include <algorithm>
#include <xpp_talos/taloslegs_inverse_kinematics.h>
#include <xpp_states/cartesian_declarations.h>

#define IKFAST_CLIBRARY
#define IKFAST_NO_MAIN

#include <xpp_talos/ikfast_left.h>
#include <xpp_talos/ikfast_right.h>

const int SOLUTION = 2;

namespace xpp {

  Eigen::VectorXd 
  TalosLegsInverseKinematics::GetJointAngles(const TalosLeg leg,
					     const Vector3d& ee_pos_H,
					     const Matrix3d& ee_or) const
  {
    // Joint angles to be calculated
    Eigen::VectorXd q(6); q.setZero();

    switch (leg) {
    case LEFTLEG:
      {
	// Solution container, stores all possible solutions
	using namespace ikfast_left_leg;
	ikfast::IkSolutionList<IkReal> solutions;
	// Representation for IK solver
	IkReal eerot[9], eetrans[3];
	bool bSuccess = false;
	// Create en effector desired 4x4 matrix (last row is 0 0 0 1 always)
	eerot[0] = ee_or(0,0); eerot[1] = ee_or(0,1); eerot[2] = ee_or(0,2); eetrans[0] = ee_pos_H.x();
	eerot[3] = ee_or(1,0); eerot[4] = ee_or(1,1); eerot[5] = ee_or(1,2); eetrans[1] = ee_pos_H.y();
	eerot[6] = ee_or(2,0); eerot[7] = ee_or(2,1); eerot[8] = ee_or(2,2); eetrans[2] = ee_pos_H.z();
	bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

	// Failed to get a solution, return 0 for all joints
	if( !bSuccess ) {
	  fprintf(stderr,"Failed to get ik solution\n");
	  return q;
	}

	// Pick a solution
	const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(SOLUTION);
	std::vector<IkReal> qv(GetNumJoints());
	sol.GetSolution(q.data(), NULL);

	return q;
      }
      break;
    case RIGHTLEG:
      {
	using namespace ikfast_right_leg;
	// Solution container, stores all possible solutions
	ikfast::IkSolutionList<IkReal> solutions;
	// Representation for IK solver
	IkReal eerot[9], eetrans[3];
	bool bSuccess = false;
	// Create en effector desired 4x4 matrix (last row is 0 0 0 1 always)
	eerot[0] = ee_or(0,0); eerot[1] = ee_or(0,1); eerot[2] = ee_or(0,2); eetrans[0] = ee_pos_H.x();
	eerot[3] = ee_or(1,0); eerot[4] = ee_or(1,1); eerot[5] = ee_or(1,2); eetrans[1] = ee_pos_H.y();
	eerot[6] = ee_or(2,0); eerot[7] = ee_or(2,1); eerot[8] = ee_or(2,2); eetrans[2] = ee_pos_H.z();
	bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);

	// Failed to get a solution, return 0 for all joints
	if( !bSuccess ) {
	  fprintf(stderr,"Failed to get ik solution\n");
	  return q;
	}

	// Pick a solution
	const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(SOLUTION);
	std::vector<IkReal> qv(GetNumJoints());
	sol.GetSolution(q.data(), NULL);

	return q;
      }
      break;
    default:
      break;
    }

    return q;
  }
}
