#ifndef PHYSICSROBOT_H
#define PHYSICSROBOT_H
#include "physicsworld.h"

class PhysicsRobot
{
public:
    PhysicsRobot();

    void add_joint(PhysicsJoint * joint);
    Eigen::Affine3d get_transform_from_base_to_link_end(int link_id);

    PhysicsObject * base;
    std::vector<PhysicsJoint*> joints;
    int numLinks{0};

};

#endif // PHYSICSROBOT_H
