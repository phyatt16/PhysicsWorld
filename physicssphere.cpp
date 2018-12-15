#include "physicssphere.h"
#include <iostream>

PhysicsSphere::PhysicsSphere(double sphereRadius, double sphereMass)
{
    this->shape = "sphere";
    radius = sphereRadius;
    mass = sphereMass;
    inertiaTensor(0,0) = 2.0/5.0*mass*pow(radius,2);
    inertiaTensor(1,1) = 2.0/5.0*mass*pow(radius,2);
    inertiaTensor(2,2) = 2.0/5.0*mass*pow(radius,2);
}
