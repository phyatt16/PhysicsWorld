#ifndef PHYSICSCYLINDER_H
#define PHYSICSCYLINDER_H

#include "physicsobject.h"


class PhysicsCylinder : public PhysicsObject
{
public:
    PhysicsCylinder();
    ~PhysicsCylinder(){};
    float height;
    float radius;
};

#endif // PHYSICSCYLINDER_H
