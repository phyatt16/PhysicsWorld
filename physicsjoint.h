#ifndef PHYSICSJOINT_H
#define PHYSICSJOINT_H

#include "physicsobject.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class PhysicsJoint
{
public:
    PhysicsJoint();

    void set_joint_angle(float angle);
    void update_joint_kinematics();
    void update_child_position_relative_to_parent();
    Eigen::Affine3d get_transform_from_parent_CoM_to_child_CoM();



    PhysicsObject * parent;
    PhysicsObject * child;
    Eigen::Affine3d TransformFromParentCoMToJoint;
    Eigen::Affine3d TransformFromJointToChildCoM;

    Eigen::Affine3d jointRotationTransform;
    Eigen::Vector3d rotationAxis;
    float jointAngle{0};

};

#endif // PHYSICSJOINT_H
