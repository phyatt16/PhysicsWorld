#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H
#include "physicsvector.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class PhysicsObject
{
public:
    PhysicsObject();
    virtual ~PhysicsObject(){};
    Eigen::Affine3d pose;
    Eigen::Matrix3d inertiaTensor;
    float mass{1.f};
    Eigen::Vector3d velocity;
    float mCoefficientOfRestitution{.8f};
    std::string shape;
};

#endif // PHYSICSOBJECT_H
