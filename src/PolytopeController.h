#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/DCMTask.h>
#include <mc_tasks/DCM_VRPTask.h>
#include <mc_tasks/VRPTask.h>
// #include "utils/MCStabilityPolytope.h"
#include <mc_dynamic_polytopes/DynamicPolytope.h>

#include <eigen-quadprog/QuadProg.h>

#include "api.h"

struct PolytopeController_DLLAPI PolytopeController : public mc_control::fsm::Controller
{
  PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  // legacy function to update the contact set for stabiliplus balance computation
  void updateContactSet(const std::vector<mc_rbdyn::Contact> & contacts, unsigned int robotIndex);
  // legacy function to update mcstabilitypolytope (wrapper for stabiliplus)
  // void updateObjective(MCStabilityPolytope * polytope_, Eigen::Vector3d currentPos, Eigen::Vector3d & objective);

private:
  sva::PTransformd wallPose_;
  sva::PTransformd robotDCMtarget_;
  bool firstPolyOK_ = false;

public:
  std::map<std::string, mc_rbdyn::Contact &> R1Contacts_;
  std::map<std::string, mc_rbdyn::Contact &> R2Contacts_;
  // stabiliplus elements
  // std::shared_ptr<MCStabilityPolytope> robotPolytope_;
  // std::shared_ptr<ContactSet> contactSet_;

  // mc_dynamic_polytopes element
  std::shared_ptr<mc_dynamic_polytopes::DynamicPolytope> DCMPoly_;
  std::shared_ptr<mc_dynamic_polytopes::DynamicPolytope> DCMPoly2_;

  std::shared_ptr<mc_tasks::DCM_VRP::DCM_VRPTask> DCMTask_;
  std::shared_ptr<mc_tasks::DCM_VRP::DCMTask> DCMFunction_;
  std::shared_ptr<mc_tasks::DCM_VRP::VRPTask> VRPFunction_;
  std::shared_ptr<mc_tasks::DCM_VRP::VRPTask> VRPFunction2_;

private:
  // planes normals and offsets for eCMP region testing and constraint
  Eigen::MatrixX3d eCMPPlanesNormals_;
  Eigen::VectorXd eCMPPlanesOffsets_;

  Eigen::QuadProgDense testSolver_;
};
