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

void PhysicsWorld::add_robot_to_world(PhysicsRobot* robot, PhysicsPIDController* PID)
{
    mRobots.push_back(robot);
    mControllers.push_back(PID);
    mNumberOfRobots++;
}


void PhysicsWorld::simulate_one_timestep(double dt, bool gravityCompensation)
{
    for(int i{0}; i<mNumberOfRobots; i++)
    {
        Eigen::VectorXd tau = mControllers[i]->get_control_inputs(mRobots[i]->q,mRobots[i]->qd);
        if(gravityCompensation)
        {
            tau += mRobots[i]->get_gravity_torques(mRobots[i]->q,g);
        }

        Eigen::VectorXd qdd = mRobots[i]->get_accel(mRobots[i]->q,mRobots[i]->qd,tau,g);
        bool noNans{true};
        for(int j{0}; j<mRobots[i]->numLinks; j++)
        {
            if(std::isnan(qdd(i))){noNans=false;}
            if(qdd(i)>10){qdd(i)=10;}
            if(qdd(i)<-10){qdd(i)=-10;}
        }
        if(noNans)
        {
            Eigen::VectorXd qd = .99*mRobots[i]->qd + 0.5*(qdd + mRobots[i]->qdd)*dt;
            mRobots[i]->qdd = qdd;

            mRobots[i]->q = mRobots[i]->q + 0.5*(qd + mRobots[i]->qd)*dt;
            mRobots[i]->qd = qd;
        }
        else
        {
            Eigen::VectorXd qd = .99*mRobots[i]->qd;

            mRobots[i]->q = mRobots[i]->q + 0.5*(qd + mRobots[i]->qd)*dt;
            mRobots[i]->qd = qd;
        }
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
