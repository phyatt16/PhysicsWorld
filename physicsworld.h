#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H
#include "physicsvector.h"
#include "physicssphere.h"

#include <vector>

class PhysicsWorld
{
public:
    PhysicsWorld();
    ~PhysicsWorld();
    PhysicsVector g;
    int get_number_of_objects();
    void add_object_to_world(PhysicsSphere*);
    PhysicsSphere * get_object(int objectId);
    void simulate_one_timestep(float dt);
private:
    int mNumberOfObjects{0};
    std::vector<PhysicsSphere *> mObjects;
    float mWorldCubeSize{5};
};

#endif // PHYSICSWORLD_H
