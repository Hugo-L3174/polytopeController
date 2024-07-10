#include "DynamicPolytope.h"

DynamicPolytope::DynamicPolytope(const std::string & name, std::set<std::string> contactNames)
: name_(name), possibleContacts_(contactNames)
{
  // Init dimension
  Rn::setDimension(3);
  // init triangles and cones maps
  for(const auto contact : contactNames)
  {
    std::vector<std::array<Eigen::Vector3d, 3UL>> newTrianglesArray;
    polytopeTrianglesMap_.emplace(contact, newTrianglesArray);

    boost::shared_ptr<Polytope_Rn> newCone(new Polytope_Rn());
    frictionCones_.emplace(contact, newCone);
  }
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
    std::pair<std::pair<double, double>, sva::PTransformd> & contactSurface,
    double m_frictionCoef)
{
  double dim = 3;
  Polytope_Rn new3dForceCone;
  // newCone
  // for now generate cone generates only the directions for the rays: we assume it is a polyhedral cone
  auto generators = generatePolyhedralConeGens(numberOfFrictionSides, contactSurface.second.rotation(), m_frictionCoef);
  // here we manipulate polytope objects so need to add origin as a generator on the polyhedral cone
  generators.emplace_back(Eigen::Vector3d::Zero());
  for(const auto g : generators)
  {
    boost::shared_ptr<Generator_Rn> gn(new Generator_Rn(dim));
    boost::numeric::ublas::vector<double> coords(3);
    coords.insert_element(0, g.x());
    coords.insert_element(1, g.y());
    coords.insert_element(2, g.z());
    gn->setCoordinates(coords);
    new3dForceCone.addGenerator(gn);
    // mc_rtc::log::info("Creating cone with vertex {}", g.transpose());
  }
  // mc_rtc::log::info("Created cone of dim {} with {} generators", new3dForceCone.dimension(),
  //                   new3dForceCone.numberOfGenerators());
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
  Eigen::MatrixXd gen = compute6DGeneratorsMatrixSingleCone(com, 6, contactSurface, 0.7);
  // build polytope object from these generators
  boost::shared_ptr<Polytope_Rn> newPoly(new Polytope_Rn());
  // for(size_t i = 0; i < count; i++)
  // {
  //   // TODO add generators matrix line per matrix line (check dim actually idk if lin or col)
  //   newPoly->addGenerator()
  // }
}

void DynamicPolytope::computeConesFromContactSet(const mc_rbdyn::Robot & robot)
{
  for(const auto contactName : activeContacts_)
  {
    mc_tasks::lipm_stabilizer::internal::Contact newContact(robot, contactName, 0.7);
    std::pair<std::pair<double, double>, sva::PTransformd> cont(
        std::pair<double, double>(newContact.halfLength(), newContact.halfWidth()), newContact.surfacePose());

    // update the correct cone in the map
    frictionCones_.at(contactName).reset(new Polytope_Rn(buildForceConeFromContact(6, cont, newContact.friction())));
    // update faces of the cone
    DoubleDescriptionFromGenerators::Compute(frictionCones_.at(contactName), 100);
    // mc_rtc::log::info("Cone for {} has {} generators and {} facets", contactName,
    //                   frictionCones_.at(contactName)->numberOfGenerators(),
    //                   frictionCones_.at(contactName)->numberOfHalfSpaces());
  }
}

// void DynamicPolytope::computeResultHull()
// {
//   // instanciation runs the whole algorithm
//   // QuickHullAlgorithm convexHull(CWC_);
// }

void DynamicPolytope::updateTrianglesGUIPolitopix()
{
  for(const auto contact : activeContacts_)
  {
    updateTrianglesPolitopix(frictionCones_.at(contact), polytopeTrianglesMap_.at(contact));
  }
  for(const auto contact : contactsToRemove_)
  {
    clearTriangles(polytopeTrianglesMap_.at(contact));
  }
}

void DynamicPolytope::updateTrianglesPolitopix(boost::shared_ptr<Polytope_Rn> & polytope,
                                               std::vector<std::array<Eigen::Vector3d, 3>> & resultTriangles)
{
  // XXX mutex?
  // XXX CAREFUL here we fill a triangles list: this assumes we are in 3d (because each facet has dim vertices in a
  // polytope) BUT this means if we actually manipulate a 6d space the faces will be hexagons? can we assume the
  // generators are made of 2 3d matrices?
  resultTriangles.clear();
  // Assuming the given polytope is already computed, get the generators for each facet, then use their coordinates to
  // create the faces in mc_rtc format.

  // This fills the list with vectors of the ids of the generators that compose each face (Never used)
  // std::vector<std::vector<unsigned int>> listOfGeneratorsPerFacet;
  // polytope->getGeneratorsPerFacet(listOfGeneratorsPerFacet);

  resultTriangles.reserve(polytope->numberOfHalfSpaces());
  // For each half space in the polytope, get the generators that compose it
  constIteratorOfListOfGeometricObjects<boost::shared_ptr<HalfSpace_Rn>> halfSpaceIter(polytope->getListOfHalfSpaces());
  constIteratorOfListOfGeometricObjects<boost::shared_ptr<Generator_Rn>> generatorIter(polytope->getListOfGenerators());

  // get a point that we know is inside the polytope to compute the faces normals
  boost::numeric::ublas::vector<double> insidePoint(3);
  TopGeomTools::gravityCenter(polytope, insidePoint);
  // recast it in eigen for practical reasons
  Eigen::Vector3d inside(insidePoint[0], insidePoint[1], insidePoint[2]);

  for(halfSpaceIter.begin(); halfSpaceIter.end() != true; halfSpaceIter.next())
  {
    // mc_rtc::log::info("Face index is {}, it has {} generators", halfSpaceIter.currentIteratorNumber(),
    //                   listOfGeneratorsPerFacet.at(halfSpaceIter.currentIteratorNumber()).size());
    std::vector<Eigen::Vector3d> vertices;
    for(generatorIter.begin(); generatorIter.end() != true; generatorIter.next())
    {

      // each generator has several facets, so iterate his facets until finding the current one
      for(size_t i = 0; i < generatorIter.current()->numberOfFacets(); i++)
      {
        if(generatorIter.current()->getFacet(i) == halfSpaceIter.current())
        {
          // mc_rtc::log::info("Generator {} belongs to face {}, adding it.", generatorIter.currentIteratorNumber(),
          // halfSpaceIter.currentIteratorNumber());
          // This generator is one of the current halfspace vertex
          // XXX here the index of the coordinate depends if we say 3d force polytope or directly 6d wrench polytope
          Eigen::Vector3d vertex(generatorIter.current()->getCoordinate(0), generatorIter.current()->getCoordinate(1),
                                 generatorIter.current()->getCoordinate(2));
          vertices.push_back(vertex);
          // mc_rtc::log::info("Vertex coords: {}", vertex.transpose());
        }
      }
    }
    /* we got all vertices of the face in vertices vector, now order them for triangle array, ie order vertices so that
    the normal is towards the exterior
    We don't necessarily have only 3 vertices for this face! if not, more calculations are necessary to decompose
    the face into triangles
    we assume the vertices were ordered + faces are convex: then we decompose into triangles by taking the first
    vertex and making a face with the two neighbors until the second neighbor is the last vertex
    */
    auto nbVertices = vertices.size();
    for(auto i = 0; i < nbVertices - 2; i++)
    {
      // mc_rtc::log::info("Making a triangle with vertices {}, {} and {}", 0, i+1, i+2);
      // make a triangle with vertices 0, i+1, i+2 and orient and emplace it normally
      Eigen::Vector3d faceNormal;
      faceNormal = (vertices.at(i + 1) - vertices.at(0)).cross(vertices.at(i + 2) - vertices.at(0));
      faceNormal.normalize();

      auto faceOffset = halfSpaceIter.current()->getConstant();

      // testing for normal direction: if inside point of the face * normal - face offset > 0 then we need to invert
      // the face
      if(inside.dot(faceNormal) - faceOffset < 0.0)
      {
        resultTriangles.push_back({vertices.at(0), vertices.at(i + 1), vertices.at(i + 2)});
      }
      else
      {
        resultTriangles.push_back({vertices.at(0), vertices.at(i + 2), vertices.at(i + 1)});
      }
    }
  }
}

void DynamicPolytope::addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
{
  category.push_back(name_);
  auto conesCat = category;
  conesCat.push_back("Friction cones");

  for(const auto contact : possibleContacts_)
  {
    gui.addElement(
        this, conesCat,
        mc_rtc::gui::Polyhedron(contact, polyForceConfig_, [this, contact]() { return getPolyTriangles(contact); }));
  }

  // gui.addElement(this, conesCat,
  //                 // mc_rtc::gui::Polyhedron(fmt::format("{} balance region", name_), polyForceConfig_,
  //                 //                        [this]() { return getPolyTriangles("LeftFootCenter"); }),
  //                 mc_rtc::gui::Polyhedron(fmt::format("{} cone", name_), polyForceConfig_,
  //                                        [this]() { return getPolyTriangles("RightFootCenter"); }),
  //                 mc_rtc::gui::Polyhedron("LeftFoot Cone", polyForceConfig_,
  //                                        [this]() { return getPolyTriangles("LeftFootCenter"); }),
  //                 mc_rtc::gui::Polyhedron("RightHand Cone", polyForceConfig_,
  //                                        [this]() { return getPolyTriangles("RightHand"); }),
  //                 mc_rtc::gui::Polyhedron("LeftHand Cone", polyForceConfig_,
  //                                        [this]() { return getPolyTriangles("LeftHand"); }));
}
