#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H
#include "physicsvector.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class PhysicsObject
{
public:
    PhysicsObject();
    virtual ~PhysicsObject(){};
    Eigen::Matrix4d pose;
    Eigen::Matrix3d inertiaTensor;
    float mass{1.f};
    float dragCoefficient{0.25f};
    PhysicsVector position{0.f,0.f,0.f};
    PhysicsVector velocity{0.f,0.f,0.f};
    float mCoefficientOfRestitution{.8f};
    std::string shape;
};

#endif // PHYSICSOBJECT_H
