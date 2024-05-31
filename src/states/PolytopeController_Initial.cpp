#include "PolytopeController_Initial.h"

#include "../PolytopeController.h"

void PolytopeController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void PolytopeController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PolytopeController &>(ctl_);
}

bool PolytopeController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PolytopeController &>(ctl_);
  output("OK");
  return true;
}

void PolytopeController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PolytopeController &>(ctl_);
}

EXPORT_SINGLE_STATE("PolytopeController_Initial", PolytopeController_Initial)
