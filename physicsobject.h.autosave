#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class PhysicsObject
{
public:
    PhysicsObject();
    virtual ~PhysicsObject(){};
    Eigen::Affine3d pose;
    Eigen::Matrix3d inertiaTensor;
    double mass{1};
    Eigen::Vector3d velocity;
    double mCoefficientOfRestitution{.8};
    std::string shape;

};

#endif // PHYSICSOBJECT_H
