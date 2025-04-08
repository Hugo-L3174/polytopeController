#include "EstimateMass.h"

#include "../PolytopeController.h"

void EstimateMass::configure(const mc_rtc::Configuration & config) {}

void EstimateMass::start(mc_control::fsm::Controller & ctl_)
{
  iterationCounter_ = 0;
  appliedWrenches_ = sva::ForceVecd::Zero();
}

bool EstimateMass::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PolytopeController &>(ctl_);

  for(const auto & contact : ctl.R1Contacts_)
  {
    auto X_0_C = sva::PTransformd(ctl.realRobot().com());
    // transform from contact frame to CoM frame (world orientation)
    auto X_r_C = X_0_C * ctl.realRobot().frame(contact.first).position().inv();
    auto contactWrench = ctl.realRobot().indirectSurfaceForceSensor(contact.first).wrench();
    // computing contact wrench back into CoM frame
    appliedWrenches_ += X_r_C.dualMul(contactWrench);
  }

  iterationCounter_++;
  if(iterationCounter_ < 100)
  {
    return false;
  }
  else
  {
    appliedWrenches_ /= iterationCounter_;
    auto estimatedMass = appliedWrenches_.force().z() / 9.81;
    ctl.DCMTask_->setRobotMass(estimatedMass);
    // ctl.solver().addTask(ctl.DCMTask_);
    output("OK");
    return true;
  }
}

void EstimateMass::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PolytopeController &>(ctl_);
}

EXPORT_SINGLE_STATE("EstimateMass", EstimateMass)
