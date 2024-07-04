#include "DynamicPolytope.h"

DynamicPolytope::DynamicPolytope(const std::string & name) : name_(name)
{
  // Init GIWC pointer
  Rn::setDimension(3);
}

DynamicPolytope::~DynamicPolytope() {}

void DynamicPolytope::load(const mc_rtc::Configuration & config)
{
  if(auto gui = config.find("polyhedronForce"))
  {
    polyForceConfig_.fromConfig(*gui);
  }
  if(auto gui = config.find("polyhedronMoment"))
  {
    polyMomentConfig_.fromConfig(*gui);
  }
}

Polytope_Rn DynamicPolytope::buildForceConeFromContact(
    int numberOfFrictionSides,
    std::pair<std::pair<double, double>, sva::PTransformd> contactSurface,
    double m_frictionCoef)
{
  double dim = 3;
  Polytope_Rn new3dForceCone;
  // newCone

  auto generators = generateCone(numberOfFrictionSides, contactSurface.second.rotation(), m_frictionCoef);
  for(auto g : generators)
  {
    boost::shared_ptr<Generator_Rn> gn(new Generator_Rn(dim));
    boost::numeric::ublas::vector<double> coords(3);
    coords.insert_element(0, g.x());
    coords.insert_element(1, g.y());
    coords.insert_element(2, g.z());
    gn->setCoordinates(coords);
    new3dForceCone.addGenerator(gn);
    mc_rtc::log::info("Creating cone with vertex {}", g.transpose());
  }
  mc_rtc::log::info("Created cone of dim {} with {} generators", new3dForceCone.dimension(),
                    new3dForceCone.numberOfGenerators());
  return new3dForceCone;
}

void DynamicPolytope::addContactWrenchCone(const mc_rbdyn::Robot & robot,
                                           const std::string & surfaceName,
                                           double friction)
{
  // create contact object (just practical for half length and width, could be done without lipm header)
  mc_tasks::lipm_stabilizer::internal::Contact newContact(robot, surfaceName, friction);
  std::pair<std::pair<double, double>, sva::PTransformd> contactSurface(
      std::pair<double, double>(newContact.halfLength(), newContact.halfWidth()), newContact.surfacePose());
  // XXX check if normal to compute generators matrix for several contacts? makes sense only if R-rep, not if V-rep and
  // bounded polytopes
  auto com = robot.com();
  // compute 6d generators matrix for 6d cone of this contact (6d vectors are columns)
  Eigen::MatrixXd gen = computeGeneratorsMatrixSingleCone(com, 6, contactSurface, 0.7);
  // build polytope object from these generators
  boost::shared_ptr<Polytope_Rn> newPoly(new Polytope_Rn());
  // for(size_t i = 0; i < count; i++)
  // {
  //   // TODO add generators matrix line per matrix line (check dim actually idk if lin or col)
  //   newPoly->addGenerator()
  // }
}

void DynamicPolytope::computeResultHull()
{
  // instanciation runs the whole algorithm
  // QuickHullAlgorithm convexHull(CWC_);
}

void DynamicPolytope::updateTriangles(const boost::shared_ptr<Polytope_Rn> & polytope,
                                      std::vector<std::array<Eigen::Vector3d, 3>> & resultTriangles)
{
  // XXX mutex?
  // XXX CAREFUL here we fill a triangles list: this assumes we are in 3d (because each facet has dim vertices in a
  // polytope) BUT this means if we actually manipulate a 6d space the faces will be hexagons? can we assume the
  // generators are made of 2 3d matrices?
  resultTriangles.clear();
  // DoubleDescriptionFromGenerators::Compute(polytope);
  // Assuming the given polytope is already computed, get the generators for each facet, then use their coordinates to
  // create the faces in mc_rtc format This fills the list with vectors of the ids of the generators that compose each
  // face
  std::vector<std::vector<unsigned int>> listOfGeneratorsPerFacet;
  polytope->getGeneratorsPerFacet(listOfGeneratorsPerFacet);

  resultTriangles.reserve(polytope->numberOfHalfSpaces());
  // For each half space in the polytope, get the generators that compose it
  constIteratorOfListOfGeometricObjects<boost::shared_ptr<HalfSpace_Rn>> halfSpaceIter(polytope->getListOfHalfSpaces());
  constIteratorOfListOfGeometricObjects<boost::shared_ptr<Generator_Rn>> generatorIter(polytope->getListOfGenerators());
  for(halfSpaceIter.begin(); halfSpaceIter.end() != true; halfSpaceIter.next())
  {
    mc_rtc::log::info("Face index is {}", halfSpaceIter.currentIteratorNumber());
    std::vector<Eigen::Vector3d> vertices;
    for(generatorIter.begin(); generatorIter.end() != true; generatorIter.next())
    {

      // each generator has several facets, so iterate his facets until finding the current one
      for(size_t i = 0; i < generatorIter.current()->numberOfFacets(); i++)
      {
        if(generatorIter.current()->getFacet(i) == halfSpaceIter.current())
        {
          mc_rtc::log::info("This generator belongs to the current face, adding it.");
          // This generator is one of the current halfspace vertex
          // XXX here the index of the coordinate depends if we say 3d force polytope or directly 6d wrench polytope

          Eigen::Vector3d vertex(generatorIter.current()->getCoordinate(0), generatorIter.current()->getCoordinate(1),
                                 generatorIter.current()->getCoordinate(2));
          vertices.push_back(vertex);
          mc_rtc::log::info("Vertex coords: {}", vertex.transpose());
        }
      }
    }
    // we got all vertices of the face in vertices vector, now order them for triangle array, ie order vertices so that
    // normal faces exterior
    // XXX check that this is the normal and not - the normal
    Eigen::Vector3d faceNormal;
    faceNormal = (vertices.at(2) - vertices.at(0)).cross(vertices.at(1) - vertices.at(0));
    faceNormal.normalize();

    if(((vertices.at(1) - vertices.at(0)).cross(vertices.at(2) - vertices.at(0))).dot(faceNormal) > 0.0)
    {
      resultTriangles.push_back({vertices.at(0), vertices.at(1), vertices.at(2)});
    }
    else
    {
      resultTriangles.push_back({vertices.at(0), vertices.at(2), vertices.at(1)});
    }
  }
}

void DynamicPolytope::addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
{
  category.push_back("Polyhedrons");

  gui.addElement(this, category,
                 mc_rtc::gui::Polyhedron(fmt::format("{} balance region", name_), polyForceConfig_,
                                         [this]() { return getPolyTriangles(); }));
}
