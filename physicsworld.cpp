#include "physicsworld.h"
#include <iostream>
#include <math.h>

PhysicsWorld::PhysicsWorld()
{
}

PhysicsWorld::~PhysicsWorld()
{
    for(int i{0}; i<mNumberOfObjects; i++)
    {
        delete mObjects[i];
    }
}

int PhysicsWorld::get_number_of_objects()
{
    return mNumberOfObjects;
}

void PhysicsWorld::add_object_to_world(PhysicsSphere* object)
{
    mObjects.push_back(object);
    mNumberOfObjects++;
}

void PhysicsWorld::set_fluid_density(float density)
{
    fluidDensity = density;
}

PhysicsSphere * PhysicsWorld::get_object(int objectId)
{
    if(objectId <= mNumberOfObjects)
    {
        return mObjects[objectId];
    }
    else
    {
        return nullptr;
    }

}

void PhysicsWorld::detect_and_simulate_wall_collision(PhysicsSphere * object)
{
    if(fabs(object->position.x) > mWorldCubeSize - object->radius)
    {
        object->velocity.x = -object->velocity.x;
        object->velocity = object->velocity*object->mCoefficientOfRestitution;

    }
    if(fabs(object->position.y) > mWorldCubeSize - object->radius)
    {
        object->velocity.y = -object->velocity.y;
        object->velocity = object->velocity*object->mCoefficientOfRestitution;
    }
    if(fabs(object->position.z) > mWorldCubeSize - object->radius)
    {
        object->velocity.z = -object->velocity.z;
        object->velocity = object->velocity*object->mCoefficientOfRestitution;
    }

    object->position.floor(-mWorldCubeSize + object->radius);
    object->position.ceil(mWorldCubeSize - object->radius);
}

PhysicsVector PhysicsWorld::calculate_drag_force_on_object(PhysicsSphere * object)
{
    float pi{3.14159};
    float dragForceMagnitude = 0.5 * fluidDensity * pow(object->velocity.norm(),2.f) * object->dragCoefficient * pi * pow(object->radius,2.f);
    PhysicsVector dragForce{ -(object->velocity)*(1.f/object->velocity.norm()) * dragForceMagnitude};

    return dragForce;
}

void PhysicsWorld::simulate_one_timestep(float dt)
{
    for(int i{0}; i<mNumberOfObjects; i++)
    {
        PhysicsVector dragForce{calculate_drag_force_on_object(mObjects[i])};

        PhysicsVector acceleration{g + dragForce*(1.f/mObjects[i]->mass)};

        if(isnan(dragForce.x))
        {
            std::cout<<"dragForce is nan"<<std::endl;
        }
        if(isnan((1.f/mObjects[i]->mass)))
        {
            std::cout<<"1/mass is nan"<<std::endl;
        }

        mObjects[i]->velocity = mObjects[i]->velocity + acceleration*dt;
        mObjects[i]->position = mObjects[i]->position + mObjects[i]->velocity*dt;


        detect_and_simulate_wall_collision(mObjects[i]);
    }
}
