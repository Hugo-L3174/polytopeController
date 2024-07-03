#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/clock.h>
#include <mc_rtc/gui.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>

#include "WrenchCones.h"

#include "../../../politopix/trunk/PolyhedralAlgorithms_Rn.h"
#include "../../../politopix/trunk/PrismaticPolyhedron_Rn.h"
#include "../../../politopix/trunk/politopixAPI.h"

struct DynamicPolytope
{
  DynamicPolytope(const std::string & name);
  ~DynamicPolytope();

  // Creates a 6d contact friction cone from the contact surface border points
  // The generators are computed then used to build the Polytope_Rn object which is added to the cones vector
  void addContactWrenchCone(const mc_rbdyn::Robot & robot, const std::string & surfaceName, double friction);

  // Updates the faces vector used for polytope display
  void updateTriangles(const boost::shared_ptr<Polytope_Rn> & polytope,
                       std::vector<std::array<Eigen::Vector3d, 3>> & resultTriangles);

  std::vector<std::array<Eigen::Vector3d, 3>> getPolyTriangles()
  {
    return polytopeTriangles_;
  };

  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {"DynamicPolytopes"});
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix = "DynamicPolytopes");
  void removeFromLogger(mc_rtc::Logger & logger);

protected:
  string name_;
  std::vector<boost::shared_ptr<Polytope_Rn>> frictionCones_;
  boost::shared_ptr<Polytope_Rn> GIWC_;
  std::vector<std::array<Eigen::Vector3d, 3>> polytopeTriangles_;
};
