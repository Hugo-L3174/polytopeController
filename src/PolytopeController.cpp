#include "PolytopeController.h"
#include <mc_tasks/MetaTaskLoader.h>

PolytopeController::PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  datastore().make_call(
      "KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot)
      { return sva::interpolate(robot.surfacePose("LeftFootCenter"), robot.surfacePose("RightFootCenter"), 0.5); });

  DCMTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::DCM_VRP::DCM_VRPTask>(solver(), config("DCM_VRPTask"));
  solver().addTask(DCMTask_);
  // initialize stabiliplus polytope object
  // robotPolytope_ = std::make_shared<MCStabilityPolytope>(robot().name());
  // robotPolytope_->load(config("StabilityPolytope")(robot().name()));
  // robotPolytope_->addToLogger(logger());
  // robotPolytope_->addToGUI(*gui());
  std::set<std::string> contactNames = {"LeftFootCenter", "RightFootCenter", "LeftHand", "RightHand"};
  DCMPoly_ = std::make_shared<DynamicPolytope>(robot().name(), contactNames, robot());
  DCMPoly_->load(config("DynamicPolytope")("mainRobot"));
  DCMPoly_->addToGUI(*gui(), 0.003);
  DCMPoly_->addToLogger(logger());

  wallPose_ = robot("wall").posW();
  wallPose_.translation() += Eigen::Vector3d(-0.05, 0.4, 1.1);
  gui()->addElement({"Wall"}, mc_rtc::gui::Transform(
                                  "centre", [this]() -> const sva::PTransformd & { return wallPose_; },
                                  [this](const sva::PTransformd & p) { wallPose_ = p; })

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
  auto ctlContacts = solver().contacts();
  std::vector<std::pair<std::string, sva::PTransformd>> contacts;
  for(auto contact : ctlContacts)
  {
    // emplacing X_0_s of target surface: will define orientation of friction cone
    // XXX NOT sufficient ! will only be the second robot world frame, not necessarily the contact frame
    // TODO take offset into account
    contacts.emplace_back(contact.r1Surface()->name(),
                          contact.r1Surface()->X_0_s(realRobots().robot(contact.r1Index())).inv());
    // contacts.emplace_back(contact.r1Surface()->name(), contact.compute_X_r2s_r1s(robots()).inv());
  }
  // set the current controller contacts for computations
  DCMPoly_->setControllerContacts(contacts);

  // get the planes to constraint or use in the controller (will be empty in the first iterations)
  DCMTask_->setDCMPoly(DCMPoly_->getVRPPlanes());
  // DCMTask_->setDCMPoly(DCMPoly_->getZeroMomentPlanes());

  for(auto & contact : contacts)
  {
    DCMTask_->setForceConesPlanes(contact.first, DCMPoly_->getConePlanes(contact.first));
  }

  return mc_control::fsm::Controller::run();
}

void PolytopeController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void PolytopeController::updateContactSet(const std::vector<mc_rbdyn::Contact> & contacts, unsigned int robotIndex)
{
  const auto & robot = robots().robot(robotIndex);
  mc_rtc::Configuration maxForces;
  if(config_.has("surfacesMaxForces"))
  {
    maxForces = config_("surfacesMaxForces");
  }

  // XXX can we avoid allocating a new contact set every time?
  contactSet_ = std::make_shared<ContactSet>(false);

  contactSet_->mass(robots().robot(robotIndex).mass());
  contactSet_->setFrictionSides(6);

  for(const auto & contact : contacts)
  {
    if(contact.r1Index() == robotIndex || contact.r2Index() == robotIndex)
    {
      const auto & surface_name =
          (contact.r1Index() == robotIndex) ? contact.r1Surface()->name() : contact.r2Surface()->name();
      const auto & surface = robot.surface(surface_name);

      const auto & bodyName = surface.bodyName();
      const auto & body_PT = robot.bodyPosW(bodyName);

      const auto & points = surface.points();
      int ptCpt = 0; // point counter
      double mu = contact.friction(); // get the friction coef h

      double fmax;
      if(maxForces.has(surface_name))
      {
        fmax = maxForces(surface_name);
      }
      else
      {
        // XXX assuming max force is mass
        fmax = robot.mass() * 9.81;
      }

      double fmin = 0; // TODO set the same for min forces?
      ContactType type;
      if(config_.has("constrainedSurfaces"))
      {
        auto constrainedSurfaces = config_("constrainedSurfaces");
        if(constrainedSurfaces.has(surface_name))
        {
          type = ContactType::constrained;
        }
        else
        {
          type = ContactType::support;
        }
      }
      else
      {
        type = ContactType::support;
      }

      for(const auto & point : points)
      {
        auto pos = body_PT.rotation().transpose() * point.translation() + body_PT.translation();

        Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
        homTrans.block<3, 3>(0, 0) = body_PT.rotation().transpose() * point.rotation().transpose();
        homTrans.block<3, 1>(0, 3) = pos;

        const auto ptName = surface.name() + "_" + std::to_string(ptCpt);
        contactSet_->addContact(ptName, homTrans, mu, fmax, fmin, type);
        ptCpt++;
      }
    }
  }

  // Adding the accelerations
  Eigen::Vector3d acceleration;

  acceleration << 0.0, 0.0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0.6, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, 0.6, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << -0.6, 0, -9.81;
  contactSet_->addCoMAcc(acceleration);

  acceleration << 0, -0.6, -9.81;
  contactSet_->addCoMAcc(acceleration);
}

void PolytopeController::updateObjective(MCStabilityPolytope * polytope_,
                                         Eigen::Vector3d currentPos,
                                         Eigen::Vector3d & objective)
{
  double filterCoeff = 0.7;
  auto planes = polytope_->constraintPlanes();

  // XXX version with objective towards middle of polytope
  Eigen::Vector3d chebichev = polytope_->chebichevCenter();
  Eigen::Vector3d bary = polytope_->baryCenter();
  // lowpass filtering of center of polytope (very noisy from every computation)
  // lowPassPolyCenter_.update(bary);
  // bary = lowPassPolyCenter_.eval();
  Eigen::Vector3d filteredObjective = (1 - filterCoeff) * currentPos + filterCoeff * bary;
  // lowPassPolyCenter_.update(chebichev);
  // chebichev = lowPassPolyCenter_.eval();
  // Eigen::Vector3d filteredObjective = (1 - chebichevCoef_) * currentPos + chebichevCoef_ * chebichev;
  // objective = polytope_.objectiveInPolytope(filteredObjective);

  // XXX version with current pos as objective
  objective = polytope_->objectiveInPolytope(currentPos);
  if(polytope_->configToChange_)
  {
    // XXX needed only if using uncolored polyhedron implementation
    // polytope_.removeFromGUI(*gui());
    // polytope_.addToGUI(*gui());
  }
  // prevent polytope projection to lower objective (from polytope form, closest point might be lower)
  if(objective.z() < currentPos.z())
  {
    objective.z() = currentPos.z();
  }
}
