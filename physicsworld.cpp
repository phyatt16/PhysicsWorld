#include "physicsworld.h"
#include <iostream>
#include <math.h>

PhysicsWorld::PhysicsWorld()
{
    this->g.z = -9.81;
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

void PhysicsWorld::add_object_to_world(PhysicsObject* object)
{
    mObjects.push_back(object);
    mNumberOfObjects++;
}




void PhysicsWorld::simulate_one_timestep(float dt)
{
    for(int i{0}; i<mNumberOfObjects; i++)
    {
        PhysicsVector acceleration{g};

        if(!std::isnan(acceleration.x) && !std::isnan(acceleration.y) && !std::isnan(acceleration.z))
        {
            mObjects[i]->velocity = mObjects[i]->velocity + acceleration*dt;
        }
        else
        {
            float velocityDampingTerm{.9};
            mObjects[i]->velocity = mObjects[i]->velocity * velocityDampingTerm;
        }

        mObjects[i]->position = mObjects[i]->position + mObjects[i]->velocity*dt;

        if(fabs(mObjects[i]->position.z) < 0)
        {
            mObjects[i]->velocity.z = -mObjects[i]->velocity.z;
            mObjects[i]->velocity = mObjects[i]->velocity*object->mCoefficientOfRestitution;
        }

    }
}
