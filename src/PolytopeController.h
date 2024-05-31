#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

#include "utils/MCStabilityPolytope.h"

#include "api.h"

struct PolytopeController_DLLAPI PolytopeController : public mc_control::fsm::Controller
{
  PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void updateContactSet(const std::vector<mc_rbdyn::Contact> & contacts, unsigned int robotIndex);
  void updateObjective(MCStabilityPolytope * polytope_, Eigen::Vector3d currentPos, Eigen::Vector3d & objective);

private:

  sva::PTransformd wallPose_;
  std::shared_ptr<MCStabilityPolytope> robotPolytope_;
  std::shared_ptr<ContactSet> contactSet_;
  bool firstPolyOK_ = false;

};