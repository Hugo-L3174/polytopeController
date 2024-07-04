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
// #include "../../../politopix/trunk/ConvexHull_Rn.h"

struct DynamicPolytope
{
  DynamicPolytope(const std::string & name);
  ~DynamicPolytope();

  Polytope_Rn buildForceConeFromContact(int numberOfFrictionSides,
                                        std::pair<std::pair<double, double>, sva::PTransformd> contactSurface,
                                        double m_frictionCoef);

  // Creates a 6d contact friction cone from the contact surface border points
  // The generators are computed then used to build the Polytope_Rn object which is added to the cones vector
  void addContactWrenchCone(const mc_rbdyn::Robot & robot, const std::string & surfaceName, double friction);

  void addContactWrenchCone(boost::shared_ptr<Polytope_Rn> & cone)
  {
    frictionCones_.emplace_back(cone);
  };

  // Computes the minkowsky sum of the given friction cones and puts the result in the CWC_ polytope object
  void computeMinkowskySum();

  // Computes the convex hull of the CWC_ polytope
  // Might be unnecessary, heavy algorithm to remove unnecessary faces
  void computeResultHull();

  // Updates the faces vector used for polytope display
  void updateTriangles(const boost::shared_ptr<Polytope_Rn> & polytope,
                       std::vector<std::array<Eigen::Vector3d, 3>> & resultTriangles);

  void resetContactSet()
  {
    frictionCones_.clear();
  };

  std::vector<std::array<Eigen::Vector3d, 3>> getPolyTriangles()
  {
    return polytopeTriangles_;
  };

  void load(const mc_rtc::Configuration & config);
  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {"DynamicPolytopes"});
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix = "DynamicPolytopes");
  void removeFromLogger(mc_rtc::Logger & logger);

protected:
  string name_;
  mc_rtc::gui::PolyhedronConfig polyForceConfig_;
  mc_rtc::gui::PolyhedronConfig polyMomentConfig_;
  std::vector<boost::shared_ptr<Polytope_Rn>> frictionCones_;
  boost::shared_ptr<Polytope_Rn> CWC_;

public:
  std::vector<std::array<Eigen::Vector3d, 3>> polytopeTriangles_;
};
