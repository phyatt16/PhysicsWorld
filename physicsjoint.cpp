#include "physicsjoint.h"


PhysicsJoint::PhysicsJoint()
{

}

void PhysicsJoint::set_joint_angle(double angle)
{
    this->jointAngle = angle;
}

void PhysicsJoint::update_joint_kinematics()
{
    this->jointRotationTransform = Eigen::AngleAxisd(jointAngle,rotationAxis);
}

Eigen::Affine3d PhysicsJoint::get_transform_from_parent_end_to_child_CoM()
{
    update_joint_kinematics();
    return jointRotationTransform*TransformFromJointToChildCoM;
}

Eigen::Affine3d PhysicsJoint::get_transform_from_parent_CoM_to_child_CoM()
{
    update_joint_kinematics();
    return TransformFromParentCoMToJoint*jointRotationTransform*TransformFromJointToChildCoM;
}

Eigen::Affine3d PhysicsJoint::get_transform_from_parent_CoM_to_child_end()
{
    update_joint_kinematics();
    return TransformFromParentCoMToJoint*jointRotationTransform*TransformFromJointToChildEnd;
}


Eigen::Affine3d PhysicsJoint::get_transform_from_parent_end_to_child_end()
{
    update_joint_kinematics();
    return jointRotationTransform*TransformFromJointToChildEnd;
}
