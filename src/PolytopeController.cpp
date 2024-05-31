#include "PolytopeController.h"

PolytopeController::PolytopeController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // initialize polytope object
  robotPolytope_ = std::make_shared<MCStabilityPolytope>(robot().name());
  robotPolytope_->load(config("StabilityPolytope")(robot().name()));
  robotPolytope_->addToLogger(logger());
  robotPolytope_->addToGUI(*gui());

  mc_rtc::log::success("PolytopeController init done ");
}

bool PolytopeController::run()
{

  auto contacts = solver().contacts();
  updateContactSet(contacts, robot().robotIndex());
  Eigen::Vector3d currentPos = robot().com();
  if(contactSet_->numberOfContacts() > 0)
  {
    // XXX check if current pos is useful (not used)
    robotPolytope_->update(contactSet_, currentPos);
  }
  firstPolyOK_ = robotPolytope_->computed();


  /* We update the objective only if the first polytope at least was computed
  Then it is updated every control iteration using the last computed polytope
  */
  if(firstPolyOK_)
  {
    // XXX this causes drift as the com follows the measured value (in choreonoid)
    // measuredDCM_ = robot().com();
    // updateObjective(robotPolytope_.get(), measuredDCM_, DCMobjective_);
  }
  else
  {
    // This is to initialize the low pass near the com and not at zero (which makes the com objective move violently in
    // the beginning otherwise) combined to chebichev coeff and cutoff period of lowpass
    // lowPassPolyCenter_.reset(robot().com());
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
  if (config_.has("surfacesMaxForces"))
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