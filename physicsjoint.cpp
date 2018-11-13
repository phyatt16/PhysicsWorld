#include "physicsjoint.h"


PhysicsJoint::PhysicsJoint()
{

}

void PhysicsJoint::set_joint_angle(float angle)
{
    this->jointAngle = angle;
}

void PhysicsJoint::update_joint_kinematics()
{
    this->jointRotationTransform = Eigen::AngleAxisd(jointAngle,rotationAxis);
}

Eigen::Affine3d PhysicsJoint::get_transform_from_parent_CoM_to_child_CoM()
{
    update_joint_kinematics();
    return TransformFromParentCoMToJoint*jointRotationTransform*TransformFromJointToChildCoM;
}

void PhysicsJoint::update_child_position_relative_to_parent()
{
    child->pose = parent->pose * get_transform_from_parent_CoM_to_child_CoM();
}
