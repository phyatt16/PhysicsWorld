#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H
#include "physicssphere.h"
#include "physicscylinder.h"
#include "physicsbox.h"
#include "physicsjoint.h"
#include "physicsobject.h"
#include "physicsrobot.h"
#include <vector>

class PhysicsWorld
{
public:
    PhysicsWorld();
    ~PhysicsWorld();
    Eigen::Vector3d g;
    int get_number_of_objects();
    void add_object_to_world(PhysicsObject*);
    void add_robot_to_world(PhysicsRobot* robot);

    PhysicsObject * get_object(int objectId);
    void simulate_one_timestep(double dt);
    std::vector<PhysicsObject *> mObjects;
    std::vector<PhysicsRobot *> mRobots;
private:
    int mNumberOfObjects{0};
    int mNumberOfRobots{0};

};



#endif // PHYSICSWORLD_H
