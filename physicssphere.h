#ifndef PHYSICSSPHERE_H
#define PHYSICSSPHERE_H
#include "physicsobject.h"

class PhysicsSphere : public PhysicsObject
{
public:
    PhysicsSphere();
    ~PhysicsSphere(){};
    double radius{0};

private:

};

#endif // PHYSICSSPHERE_H
