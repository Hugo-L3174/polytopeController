#pragma once 
#include <mc_control/fsm/Controller.h>
// #include <eigen-cdd/Polyhedron.h>

Eigen::MatrixXd linearizedFrictionCone(int numberOfFrictionSides, Eigen::Matrix3d m_rotation, double m_frictionCoef);
std::vector<Eigen::Vector3d> generateCone(int numberOfFrictionSides, Eigen::Matrix3d m_rotation, double m_frictionCoef);
Eigen::MatrixXd computeGeneratorsMatrix(Eigen::Vector3d applicationPoint, int numberOfFrictionSides, std::vector<std::pair<std::pair<double, double>, sva::PTransformd>> contacts, double m_frictionCoef);
Eigen::Matrix3d skewMatrix(const Eigen::Vector3d v);