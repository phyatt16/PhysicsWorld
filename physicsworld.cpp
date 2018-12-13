#include "physicsworld.h"
#include <iostream>
#include <math.h>

PhysicsWorld::PhysicsWorld()
{
    this->g(0) = 0;
    this->g(1) = 0;
    this->g(2) = -9.81;
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

void PhysicsWorld::add_robot_to_world(PhysicsRobot* robot)
{
    mRobots.push_back(robot);
    mNumberOfRobots++;
}


void PhysicsWorld::simulate_one_timestep(double dt)
{

    for(int i{0}; i<mNumberOfRobots; i++)
    {
        Eigen::VectorXd tau(mRobots[i]->numLinks);
        tau(i) = 0;

        mRobots[i]->qdd = mRobots[i]->get_accel(mRobots[i]->q,mRobots[i]->qd,tau,g);
        mRobots[i]->qd = mRobots[i]->qd + mRobots[i]->qdd*dt;
        mRobots[i]->q = mRobots[i]->q + mRobots[i]->qd*dt;
        std::cout<<"Simulating one timestep..."<<std::endl;
    }

    /*
    for(int i{0}; i<mNumberOfObjects; i++)
    {
        Eigen::Vector3d acceleration{g};

        if(!std::isnan(acceleration(0)) && !std::isnan(acceleration(1)) && !std::isnan(acceleration(2)))
        {
            mObjects[i]->velocity = mObjects[i]->velocity + acceleration*dt;
        }
        else
        {
            double velocityDampingTerm{.9};
            mObjects[i]->velocity = mObjects[i]->velocity * velocityDampingTerm;
        }

        mObjects[i]->pose.translation() = mObjects[i]->pose.translation() + mObjects[i]->velocity*dt;


        if(mObjects[i]->pose.translation()(2) <= 0)
        {
            mObjects[i]->velocity(2) = -mObjects[i]->velocity(2);
            mObjects[i]->velocity = mObjects[i]->velocity*mObjects[i]->mCoefficientOfRestitution;
        }
        if(mObjects[i]->pose.translation()(2)<0){mObjects[i]->pose.translation()(2)=0;}
    }
    */
}
