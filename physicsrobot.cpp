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
    //std::cout<<transformFromBaseToLinkEnd.matrix()<<std::endl;
    for(int i{1}; i<numLinks; i++)
    {
        transformFromBaseToLinkEnd = joints[i]->get_transform_from_parent_end_to_child_end()*transformFromBaseToLinkEnd;
        //std::cout<<joints[i]->get_transform_from_parent_end_to_child_end().matrix()<<std::endl;
        //std::cout<<transformFromBaseToLinkEnd.matrix()<<std::endl;

    }
    return transformFromBaseToLinkEnd;
}
