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
    float fluidDensity{0.f};
    int get_number_of_objects();
    void add_object_to_world(PhysicsSphere*);
    void set_fluid_density(float);
    PhysicsSphere * get_object(int objectId);
    void simulate_one_timestep(float dt);
private:
    int mNumberOfObjects{0};
    std::vector<PhysicsSphere *> mObjects;
    float mWorldCubeSize{5};
    void detect_and_simulate_wall_collision(PhysicsSphere *);
    PhysicsVector calculate_drag_force_on_object(PhysicsSphere *);
};

#endif // PHYSICSWORLD_H
