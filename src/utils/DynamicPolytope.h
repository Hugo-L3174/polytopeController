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

#include <eigen-cdd/Polyhedron.h>

struct DynamicPolytope
{
  DynamicPolytope(const std::string & name, std::set<std::string> contactNames);
  ~DynamicPolytope();

  Polytope_Rn buildForceConeFromContact(int numberOfFrictionSides,
                                        std::pair<std::pair<double, double>, sva::PTransformd> & contactSurface,
                                        double m_frictionCoef);

  // Creates a 6d contact friction cone from the contact surface border points
  // The generators are computed then used to build the Polytope_Rn object which is added to the cones vector
  void addContactWrenchCone(const mc_rbdyn::Robot & robot, const std::string & surfaceName, double friction);

  /* computes all cones from the surfaces with the given names (set by setCurrentContacts), reset the pointers of the
  map and updates the H-description of the poly using the double description algorithm.
  */
  void computeConesFromContactSet(const mc_rbdyn::Robot & robot);

  // Computes the minkowsky sum of the given friction cones and puts the result in the CWC_ polytope object
  void computeMinkowskySumPolitopix();

  void computeECMPRegion(Eigen::Vector3d comPosition);

  // Computes the convex hull of the CWC_ polytope
  // Might be unnecessary, heavy algorithm to remove unnecessary faces
  void computeResultHull();

  // Updates the internal maps of triangles for gui display for the given contact names
  void updateTrianglesGUIPolitopix();

  void resetContactSet()
  {
    frictionCones_.clear();
  };

  std::vector<std::array<Eigen::Vector3d, 3>> getPolyTriangles(const std::string & name)
  {
    return polytopeTrianglesMap_.at(name);
  };

  std::vector<std::array<Eigen::Vector3d, 3>> getCWCTriangles()
  {
    return CWCTriangles_;
  };

  // From the current contact set, deduce what contacts need to be removed from computation compared to last iteration
  void setCurrentContacts(std::vector<std::string> & contactNames)
  {
    // take the previous set of contacts
    contactsToRemove_ = activeContacts_;
    activeContacts_.clear();
    for(const auto contactName : contactNames)
    {
      // add every contact given to the active contacts
      activeContacts_.emplace(contactName);
      // every active contact does not need to be removed
      contactsToRemove_.erase(contactName);
    }
  };

  void load(const mc_rtc::Configuration & config);
  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {"DynamicPolytopes"});
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix = "DynamicPolytopes");
  void removeFromLogger(mc_rtc::Logger & logger);

protected:
  // Updates the faces vector used for polytope display (internal function)
  void updateTrianglesPolitopix(boost::shared_ptr<Polytope_Rn> & polytope,
                                std::vector<std::array<Eigen::Vector3d, 3>> & resultTriangles);

  void clearTriangles(std::vector<std::array<Eigen::Vector3d, 3>> & resultTriangles)
  {
    resultTriangles.clear();
  };

  std::string name_;
  std::set<std::string> possibleContacts_;
  std::set<std::string> activeContacts_;
  std::set<std::string> contactsToRemove_;

  mc_rtc::gui::PolyhedronConfig polyForceConfig_;
  mc_rtc::gui::PolyhedronConfig polyMomentConfig_;

  // politopix
  std::map<std::string, boost::shared_ptr<Polytope_Rn>> frictionCones_;
  boost::shared_ptr<Polytope_Rn> CWC_;

  // cdd
  std::vector<std::shared_ptr<Eigen::Polyhedron>> cddFrictionCones_;

  // map of polytope triangles for display
  std::map<std::string, std::vector<std::array<Eigen::Vector3d, 3>>> polytopeTrianglesMap_;
  std::vector<std::array<Eigen::Vector3d, 3>> CWCTriangles_;
};
