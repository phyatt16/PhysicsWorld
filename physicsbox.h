#ifndef PHYSICSBOX_H
#define PHYSICSBOX_H
#include "physicsobject.h"

class PhysicsBox : public PhysicsObject
{
public:
    PhysicsBox(double, double, double, double);
    ~PhysicsBox(){};
    double length;
    double width;
    double height;
};

#endif // PHYSICSBOX_H
