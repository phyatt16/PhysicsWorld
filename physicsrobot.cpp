#include "physicsrobot.h"
#include "physicsworld.h"

PhysicsRobot::PhysicsRobot()
{

}

void PhysicsRobot::add_joint(PhysicsJoint * joint)
{
    joints.push_back(joint);
    numLinks++;
}

Eigen::Affine3d PhysicsRobot::get_transform_from_base_to_link_end(int link_id)
{
    Eigen::Affine3d transformFromBaseToLinkEnd = joints[0]->get_transform_from_parent_end_to_child_end();
    for(int i{1}; i<=link_id; i++)
    {
        transformFromBaseToLinkEnd = transformFromBaseToLinkEnd*joints[i]->get_transform_from_parent_end_to_child_end();
    }
    return transformFromBaseToLinkEnd;
}

Eigen::Affine3d PhysicsRobot::get_transform_from_base_to_link_CoM(int link_id)
{
    Eigen::Affine3d transformFromBaseToLinkCoM;
    if(link_id==0)
    {
        transformFromBaseToLinkCoM = joints[0]->get_transform_from_parent_end_to_child_CoM();
    }
    else
    {
        transformFromBaseToLinkCoM = joints[0]->get_transform_from_parent_end_to_child_end();
        for(int i{1}; i<link_id; i++)
        {
            transformFromBaseToLinkCoM = transformFromBaseToLinkCoM*joints[i]->get_transform_from_parent_end_to_child_end();
        }
        transformFromBaseToLinkCoM = transformFromBaseToLinkCoM*joints[link_id]->get_transform_from_parent_end_to_child_CoM();

    }
    return transformFromBaseToLinkCoM;
}

void PhysicsRobot::update_robot_kinematics()
{
    for(int i{0}; i<numLinks; i++)
    {
        this->joints[i]->update_joint_kinematics();
        this->joints[i]->child->pose = this->joints[i]->parent->pose*this->joints[i]->get_transform_from_parent_CoM_to_child_CoM();
    }

}

PhysicsRobot create_n_link_robot(PhysicsWorld *world,int numLinks)
{
    PhysicsRobot robot;

    PhysicsBox *box = new PhysicsBox;
    box->height = .01;
    box->length = .01;
    box->width = .01;
    robot.base = box;

    for(int i{0}; i<numLinks; i++)
    {
        PhysicsCylinder *cylinder = new PhysicsCylinder;
        cylinder->height = 1.0;
        cylinder->radius = .1;
        cylinder->mass = 1;
        world->add_object_to_world(cylinder);
        PhysicsJoint * joint = new PhysicsJoint;
        if(i==0){joint->parent = robot.base;}
        else{joint->parent = robot.joints[i-1]->child;}

        joint->child = cylinder;
        joint->rotationAxis << 1, 0, 0;

        if(i==0){joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,0);}
        else{joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,cylinder->height/2.0);}

        joint->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,cylinder->height/2.0);
        joint->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,cylinder->height);
        robot.add_joint(joint);
    }

    return robot;
}
