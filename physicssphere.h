#ifndef PHYSICSSPHERE_H
#define PHYSICSSPHERE_H
#include "physicsobject.h"

class PhysicsSphere : public PhysicsObject
{
public:
    PhysicsSphere(double radius,double mass);
    ~PhysicsSphere(){};
    double radius;

private:

};

#endif // PHYSICSSPHERE_H
