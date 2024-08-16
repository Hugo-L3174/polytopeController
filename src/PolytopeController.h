#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

#include "utils/MCStabilityPolytope.h"
#include <mc_dynamic_polytopes/DynamicPolytope.h>

#include "api.h"

struct PolytopeController_DLLAPI PolytopeController : public mc_control::fsm::Controller
{
  PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  // legacy function to update the contact set for stabiliplus balance computation
  void updateContactSet(const std::vector<mc_rbdyn::Contact> & contacts, unsigned int robotIndex);
  // legacy function to update mcstabilitypolytope (wrapper for stabiliplus)
  void updateObjective(MCStabilityPolytope * polytope_, Eigen::Vector3d currentPos, Eigen::Vector3d & objective);

private:
  sva::PTransformd wallPose_;
  bool firstPolyOK_ = false;

  // stabiliplus elements
  std::shared_ptr<MCStabilityPolytope> robotPolytope_;
  std::shared_ptr<ContactSet> contactSet_;

  // mc_dynamic_polytopes element
  std::shared_ptr<DynamicPolytope> DCMPoly_;

  // planes normals and offsets for eCMP region testing and constraint
  Eigen::MatrixX3d eCMPPlanesNormals_;
  Eigen::VectorXd eCMPPlanesOffsets_;
};
