#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H
#include "physicsvector.h"
#include "physicssphere.h"
#include "physicscylinder.h"
#include "physicsbox.h"
#include "physicsjoint.h"
#include <vector>

class PhysicsWorld
{
public:
    PhysicsWorld();
    ~PhysicsWorld();
    Eigen::Vector3d g;
    int get_number_of_objects();
    void add_object_to_world(PhysicsObject*);

    PhysicsObject * get_object(int objectId);
    void simulate_one_timestep(float dt);
    std::vector<PhysicsObject *> mObjects;
private:
    int mNumberOfObjects{0};

};



#endif // PHYSICSWORLD_H
