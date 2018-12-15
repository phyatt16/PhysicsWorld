#ifndef PHYSICSJOINT_H
#define PHYSICSJOINT_H

#include "physicsobject.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class PhysicsJoint
{
public:
    PhysicsJoint();

    void set_joint_angle(double angle);
    void update_joint_kinematics();
    void update_child_position_relative_to_parent();
    Eigen::Affine3d get_transform_from_parent_end_to_child_CoM();
    Eigen::Affine3d get_transform_from_parent_CoM_to_child_CoM();
    Eigen::Affine3d get_transform_from_parent_CoM_to_child_end();
    Eigen::Affine3d get_transform_from_parent_end_to_child_end();

    PhysicsObject * parent;
    PhysicsObject * child;
    Eigen::Affine3d TransformFromParentCoMToJoint;
    Eigen::Affine3d TransformFromJointToChildCoM;
    Eigen::Affine3d TransformFromJointToChildEnd;
    Eigen::Affine3d jointRotationTransform;
    Eigen::Vector3d rotationAxis;
    double jointAngle{0};

};

#endif // PHYSICSJOINT_H
