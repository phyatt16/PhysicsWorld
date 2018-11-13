#include "physicsobject.h"

PhysicsObject::PhysicsObject()
{
    pose = Eigen::AngleAxisd(0,Eigen::Vector3d(0,0,1));
    inertiaTensor = Eigen::Matrix3d::Identity();
}
