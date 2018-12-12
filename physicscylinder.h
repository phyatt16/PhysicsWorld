#ifndef PHYSICSCYLINDER_H
#define PHYSICSCYLINDER_H

#include "physicsobject.h"


class PhysicsCylinder : public PhysicsObject
{
public:
    PhysicsCylinder(double height, double radius, double mass);
    ~PhysicsCylinder(){};
    double height;
    double radius;

};

#endif // PHYSICSCYLINDER_H
