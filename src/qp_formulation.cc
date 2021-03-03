#include <types.h>
#include <talos_wbc_controller/qp_formulation.hpp>

namespace talos_wbc_controller {

  QpFormulation::QpFormulation()
    : joint_task_weight_(0.5), Kp_(1.0), Kv_(1.0)
  {
    
  }
  
  void
  QpFormulation::SetPositionErrors(const PosErrors& ep)
  {
    ep_ = ep;
  }

  void
  QpFormulation::SetVelocityErrors(const VelErrors& ev)
  {
    ev_ = ev;
  }

  void
  QpFormulation::SetReferenceAccelerations(const AccVector& qrdd)
  {
    qrdd_ = qrdd;
  }

  void
  QpFormulation::SetKP(double Kp)
  {
    Kp_ = Kp;
  }

  void
  QpFormulation::SetKV(double Kv)
  {
    Kv_ = Kv;
  }

  QpFormulation::AccVector
  QpFormulation::GetDesiredAccelerations(void)
  {
    if (ep_.empty() or ev_.empty() or qrdd_.empty() or
	not (ep_.size() == ev_.size()) or not (ep_.size() == qrdd_.size())) return {};

    AccVector acc_desired(ep_.size());
    for (size_t i = 0; i < acc_desired.size(); ++i) {
      acc_desired[i] = qrdd_[i] + Kv_ * ev_[i] + Kp_ * ep_[i];
    }

    return acc_desired;
  }

}
