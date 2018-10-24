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
    detect_and_simulate_object_collision(object);
    detect_and_simulate_wall_collision(object);

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

void PhysicsWorld::detect_and_simulate_object_collision(PhysicsSphere * object)
{
    PhysicsVector PrimaryObjectPosition{object->position};
    for(int i{0}; i<this->get_number_of_objects(); i++)
    {
        if(this->get_object(i) != object)
        {
            PhysicsSphere * secondaryObject{this->get_object(i)};
            PhysicsVector vectorBetweenObjects{object->position - secondaryObject->position};

            float sumOfRadii{object->radius + secondaryObject->radius};

            if(vectorBetweenObjects.norm() < sumOfRadii)
            {
                float intersectionDistance{vectorBetweenObjects.norm()-sumOfRadii};
                resolve_object_collision(object,secondaryObject,vectorBetweenObjects,intersectionDistance);
            }
        }
    }
}

void PhysicsWorld::resolve_object_collision(PhysicsSphere * object1,PhysicsSphere * object2,PhysicsVector &vectorFromOneToTwo,float intersectionDistance)
{
    object1->position = object1->position + -vectorFromOneToTwo*intersectionDistance;
    object2->position = object2->position + vectorFromOneToTwo*intersectionDistance;

    PhysicsVector velocityOfOneRelativeToTwo{object1->velocity - object2->velocity};

    object1->velocity = object1->mCoefficientOfRestitution*(object1->velocity - 2*object2->mass/(object1->mass+object2->mass) * velocityOfOneRelativeToTwo.dot(vectorFromOneToTwo)*1/(powf(vectorFromOneToTwo.norm(),2.0)) * vectorFromOneToTwo);
    object2->velocity = object2->mCoefficientOfRestitution*(object2->velocity - 2*object1->mass/(object1->mass+object2->mass) * velocityOfOneRelativeToTwo.dot(vectorFromOneToTwo)*1/(powf(vectorFromOneToTwo.norm(),2.0)) * -vectorFromOneToTwo);
}

PhysicsVector PhysicsWorld::calculate_drag_force_on_object(PhysicsSphere * object)
{
    float pi{3.14159};
    float dragForceMagnitude = 0.5 * fluidDensity * pow(object->velocity.norm(),2.f) * object->dragCoefficient * pi * pow(object->radius,2.f);
    PhysicsVector dragForce{ -(object->velocity)*(1.f/object->velocity.norm()) * dragForceMagnitude};

    if(isnan(object->velocity.norm()))
    {
        std::cout<<"norm of velocity is nan"<<std::endl;
        std::cout<<object->velocity.x<<"  "<<object->velocity.y<<"  "<<object->velocity.z<<std::endl;
    }

    return dragForce;
}

void PhysicsWorld::simulate_one_timestep(float dt)
{
    for(int i{0}; i<mNumberOfObjects; i++)
    {
        PhysicsVector dragForce{calculate_drag_force_on_object(mObjects[i])};

        PhysicsVector acceleration{g + dragForce*(1.f/mObjects[i]->mass)};

        if(!isnan(acceleration.x) && !isnan(acceleration.y) && !isnan(acceleration.z))
        {
            mObjects[i]->velocity = mObjects[i]->velocity + acceleration*dt;
        }
        else
        {
            float velocityDampingTerm{.9};
            mObjects[i]->velocity = mObjects[i]->velocity * velocityDampingTerm;
        }

        mObjects[i]->position = mObjects[i]->position + mObjects[i]->velocity*dt;


        detect_and_simulate_object_collision(mObjects[i]);
        detect_and_simulate_wall_collision(mObjects[i]);
    }
}
