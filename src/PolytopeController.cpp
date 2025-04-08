#include "PolytopeController.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <optional>

PolytopeController::PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, {mc_solver::QPSolver::Backend::TVM})
{
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot & robot) {
                          return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
                        });

  DCMTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::DCM_VRP::DCM_VRPTask>(solver(), config("DCM_VRPTask"));
  // XXX Comment task while testing the rest
  // solver().addTask(DCMTask_);
  // initialize stabiliplus polytope object
  // robotPolytope_ = std::make_shared<MCStabilityPolytope>(robot().name());
  // robotPolytope_->load(config("StabilityPolytope")(robot().name()));
  // robotPolytope_->addToLogger(logger());
  // robotPolytope_->addToGUI(*gui());
  DCMPoly_ = std::make_shared<DynamicPolytope>(robot().name(), robot(), config("DynamicPolytope")("mainRobot"));
  DCMPoly_->addToGUI(*gui(), 0.001);
  DCMPoly_->addToLogger(logger());

  DCMPoly2_ = std::make_shared<DynamicPolytope>("jvrc1", robot("jvrc1"), config("DynamicPolytope")("jvrc1"));
  DCMPoly2_->addToGUI(*gui(), 0.001);
  DCMPoly2_->addToLogger(logger());

  wallPose_ = robot("wall").posW();
  wallPose_.translation() += Eigen::Vector3d(-0.05, 0.4, 1.1);
  gui()->addElement({"Wall"}, mc_rtc::gui::Transform(
                                  "centre", [this]() -> const sva::PTransformd & { return wallPose_; },
                                  [this](const sva::PTransformd & p) { wallPose_ = p; })

  );

  robotDCMtarget_ = robot().com();
  gui()->addElement({"Robot"}, mc_rtc::gui::Transform(
                                   "DCMobjective", [this]() -> const sva::PTransformd & { return robotDCMtarget_; },
                                   [this](const sva::PTransformd & p) { robotDCMtarget_ = p; })

  );

  mc_rtc::log::success("PolytopeController init done ");
}

bool PolytopeController::run()
{
  // move wall around using gui pointer
  sva::PTransformd pos = wallPose_;
  pos.translation() -= Eigen::Vector3d(-0.05, 0.4, 1.1);
  // XXX updating posW does not seem to update the contact constraints?
  // robot("wall").posW(pos);

  // get list of the current contacts
  R1Contacts_.clear();
  R2Contacts_.clear();
  for(const auto & contact : solver().contacts())
  {
    // emplacing X_r1_r2 between controlled and target contact: will define orientation of friction cone in controlled
    // frame
    // TODO take offset into account
    // TODO compute a default contact polytope even when there is no contact (transform should be identity then)
    // This should allow "testing" if there exists a valid distrib by adding one of the unused contacts where it
    // currently is
    // Should be extended by mpc but idk if computation is too heavy
    if(contact->r1Index() == DCMPoly_->robot().robotIndex())
    {
      R1Contacts_.emplace(contact->r1Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    }
    else if(contact->r2Index() == DCMPoly_->robot().robotIndex())
    {
      R1Contacts_.emplace(contact->r2Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    }
    if(contact->r1Index() == DCMPoly2_->robot().robotIndex())
    {
      R2Contacts_.emplace(contact->r1Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    }
    else if(contact->r2Index() == DCMPoly2_->robot().robotIndex())
    {
      R2Contacts_.emplace(contact->r2Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    }
  }

  // set the current controller contacts for computations
  DCMPoly_->setControllerContacts(R1Contacts_);
  DCMPoly2_->setControllerContacts(R2Contacts_);

  DCMTask_->setDCMTarget(robotDCMtarget_.translation());

  // get the planes to constraint or use in the controller (will be empty in the first iterations)
  for(auto & contact : R1Contacts_)
  {
    const auto & feasiblePolytope = DCMPoly_->getForcePolyPlanes(contact.first);
    DCMTask_->setContactPlanes(contact.first, feasiblePolytope);
  }
  DCMTask_->setDCMPoly(DCMPoly_->getVRPPlanes());
  DCMTask_->setZeroMomentPoly(DCMPoly_->getZeroMomentPlanes());

  return mc_control::fsm::Controller::run();
}

void PolytopeController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

// void PolytopeController::updateObjective(MCStabilityPolytope * polytope_,
//                                          Eigen::Vector3d currentPos,
//                                          Eigen::Vector3d & objective)
// {
//   double filterCoeff = 0.7;
//   auto planes = polytope_->constraintPlanes();

//   // XXX version with objective towards middle of polytope
//   Eigen::Vector3d chebichev = polytope_->chebichevCenter();
//   Eigen::Vector3d bary = polytope_->baryCenter();
//   // lowpass filtering of center of polytope (very noisy from every computation)
//   // lowPassPolyCenter_.update(bary);
//   // bary = lowPassPolyCenter_.eval();
//   Eigen::Vector3d filteredObjective = (1 - filterCoeff) * currentPos + filterCoeff * bary;
//   // lowPassPolyCenter_.update(chebichev);
//   // chebichev = lowPassPolyCenter_.eval();
//   // Eigen::Vector3d filteredObjective = (1 - chebichevCoef_) * currentPos + chebichevCoef_ * chebichev;
//   // objective = polytope_.objectiveInPolytope(filteredObjective);

//   // XXX version with current pos as objective
//   objective = polytope_->objectiveInPolytope(currentPos);
//   if(polytope_->configToChange_)
//   {
//     // XXX needed only if using uncolored polyhedron implementation
//     // polytope_.removeFromGUI(*gui());
//     // polytope_.addToGUI(*gui());
//   }
//   // prevent polytope projection to lower objective (from polytope form, closest point might be lower)
//   if(objective.z() < currentPos.z())
//   {
//     objective.z() = currentPos.z();
//   }
// }
