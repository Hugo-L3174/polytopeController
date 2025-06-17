#include "PolytopeController.h"
#include <mc_tasks/MetaTaskLoader.h>

PolytopeController::PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, {mc_solver::QPSolver::Backend::TVM})
{
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot & robot) {
                          return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
                        });

  // DCMTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::DCM_VRP::DCM_VRPTask>(solver(), config("DCM_VRPTask"));
  // solver().addTask(DCMTask_);

  // DCMFunction_ = mc_tasks::MetaTaskLoader::load<mc_tasks::DCM_VRP::DCMTask>(solver(), config("DCMTask"));
  // solver().addTask(DCMFunction_);

  VRPFunction_ = mc_tasks::MetaTaskLoader::load<mc_tasks::DCM_VRP::VRPTask>(solver(), config("VRPTask"));
  solver().addTask(VRPFunction_);

  // DCMPoly_ = std::make_shared<DynamicPolytope>(robot().name(), realRobot(), config("DynamicPolytope")("mainRobot"));
  DCMPoly_ = std::make_shared<DynamicPolytope>(robot().name(), robot(), config("DynamicPolytope")("mainRobot"));
  DCMPoly_->addToGUI(*gui(), 0.001);
  DCMPoly_->addToLogger(logger());

  // DCMPoly2_ = std::make_shared<DynamicPolytope>("jvrc1", robot("jvrc1"), config("DynamicPolytope")("jvrc1"));
  // DCMPoly2_->addToGUI(*gui(), 0.001);
  // DCMPoly2_->addToLogger(logger());

  robotDCMtarget_ = robot().com();
  // gui()->addElement({"Robot"}, mc_rtc::gui::Transform(
  //                                  "DCMobjective", [this]() -> const sva::PTransformd & { return robotDCMtarget_; },
  //                                  [this](const sva::PTransformd & p) { robotDCMtarget_ = p; })
  // );

  mc_rtc::log::success("PolytopeController init done ");
}

bool PolytopeController::run()
{
  // get list of the current contacts
  R1Contacts_.clear();
  R2Contacts_.clear();
  for(const auto & contact : solver().contacts())
  {
    // forced to do this otherwise never updated
    contact->compute_X_r2s_r1s(robots());
    // emplacing X_r1_r2 between controlled and target contact: will define orientation of friction cone in controlled
    // frame
    // Should be extended by mpc but idk if computation is too heavy
    // TODO handle this in library
    if(contact->r1Index() == DCMPoly_->robot().robotIndex())
    {
      R1Contacts_.emplace(contact->r1Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    }
    else if(contact->r2Index() == DCMPoly_->robot().robotIndex())
    {
      R1Contacts_.emplace(contact->r2Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    }
    // if(contact->r1Index() == DCMPoly2_->robot().robotIndex())
    // {
    //   R2Contacts_.emplace(contact->r1Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    // }
    // else if(contact->r2Index() == DCMPoly2_->robot().robotIndex())
    // {
    //   R2Contacts_.emplace(contact->r2Surface()->name(), const_cast<mc_rbdyn::Contact &>(*contact));
    // }
  }

  // set the current controller contacts for computations (comment to not run the polytope lib)
  DCMPoly_->setControllerContacts(R1Contacts_);
  // DCMPoly2_->setControllerContacts(R2Contacts_);

  // set targets for tasks (not needed if manipulated from GUI)
  // DCMTask_->setDCMTarget(robotDCMtarget_.translation());
  // DCMFunction_->targetDCM(robotDCMtarget_.translation());
  // VRPFunction_->targetDCM(robotDCMtarget_.translation());

  // get the planes to constraint or use in the controller (will be empty in the first iterations)
  // for(auto & contact : R1Contacts_)
  // {
  //   const auto & feasiblePolytope = DCMPoly_->getForcePolyPlanes(contact.first);
  //   DCMTask_->setContactPlanes(contact.first, feasiblePolytope);
  // }
  // DCMTask_->setDCMPoly(DCMPoly_->getVRPPlanes());
  // DCMTask_->setZeroMomentPoly(DCMPoly_->getZeroMomentPlanes());

  return mc_control::fsm::Controller::run();
  // return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ObservedRobots);
}

void PolytopeController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  if(DCMTask_)
  {
    DCMTask_->reset();
  }
  if(DCMFunction_)
  {
    DCMFunction_->reset();
  }
  if(VRPFunction_)
  {
    VRPFunction_->reset();
  }
}
