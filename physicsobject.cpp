#include "physicsobject.h"

PhysicsObject::PhysicsObject()
{
    pose = Eigen::Matrix4d::Identity();
    inertiaTensor = Eigen::Matrix3d::Identity();
}
