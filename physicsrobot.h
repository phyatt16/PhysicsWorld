#ifndef PHYSICSROBOT_H
#define PHYSICSROBOT_H
#include "physicsworld.h"

class PhysicsRobot
{
public:
    PhysicsRobot();

    void add_joint(PhysicsJoint * joint);
    void update_robot_kinematics();
    Eigen::Affine3d get_transform_from_base_to_link_end(int link_id);
    Eigen::Affine3d get_transform_from_base_to_link_CoM(int link_id);

    PhysicsObject * base;
    std::vector<PhysicsJoint*> joints;
    int numLinks{0};

};

PhysicsRobot create_n_link_robot(PhysicsWorld *world, int numLinks);

#endif // PHYSICSROBOT_H
