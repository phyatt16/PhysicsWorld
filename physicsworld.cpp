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

void PhysicsWorld::simulate_one_timestep(float dt)
{
    for(int i{0}; i<mNumberOfObjects; i++)
    {
        mObjects[i]->velocity = mObjects[i]->velocity + (this->g)*dt;
        mObjects[i]->position = mObjects[i]->position + mObjects[i]->velocity*dt;

        if(fabs(mObjects[i]->position.x) > mWorldCubeSize - mObjects[i]->radius)
        {
            mObjects[i]->velocity.x = -mObjects[i]->velocity.x;
            mObjects[i]->velocity = mObjects[i]->velocity*mObjects[i]->mCoefficientOfRestitution;

        }
        if(fabs(mObjects[i]->position.y) > mWorldCubeSize - mObjects[i]->radius)
        {
            mObjects[i]->velocity.y = -mObjects[i]->velocity.y;
            mObjects[i]->velocity = mObjects[i]->velocity*mObjects[i]->mCoefficientOfRestitution;
        }
        if(fabs(mObjects[i]->position.z) > mWorldCubeSize - mObjects[i]->radius)
        {
            mObjects[i]->velocity.z = -mObjects[i]->velocity.z;
            mObjects[i]->velocity = mObjects[i]->velocity*mObjects[i]->mCoefficientOfRestitution;
        }

        mObjects[i]->position.floor(-mWorldCubeSize + mObjects[i]->radius);
        mObjects[i]->position.ceil(mWorldCubeSize - mObjects[i]->radius);

        //std::cout<<"Object: "<<i<<"  Position: "<<mObjects[i]->position.x<<"  "<<mObjects[i]->position.y<<"  "<<mObjects[i]->position.z<<std::endl;
    }
}
