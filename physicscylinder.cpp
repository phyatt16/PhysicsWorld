#include "physicscylinder.h"

PhysicsCylinder::PhysicsCylinder(double height, double radius, double mass)
{
    this->shape = "cylinder";
    this->height = height;
    this->radius = radius;
    this->mass = mass;
    inertiaTensor(0,0) = mass*(3.0*pow(radius,2) +pow(height,2))/12.0;
    inertiaTensor(1,1) = mass*(3.0*pow(radius,2) +pow(height,2))/12.0;
    inertiaTensor(2,2) = mass*pow(radius,2)/2.0;
}
