#ifndef PHYSICSSPHERE_H
#define PHYSICSSPHERE_H
#include "physicsobject.h"

class PhysicsSphere : public PhysicsObject
{
public:
    PhysicsSphere();
    ~PhysicsSphere(){};
    float radius{0.f};

private:

};

#endif // PHYSICSSPHERE_H
