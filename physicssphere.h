#ifndef PHYSICSSPHERE_H
#define PHYSICSSPHERE_H
#include "physicsvector.h"

class PhysicsSphere
{
public:
    PhysicsSphere();
    float mass{0};
    float radius{0};
    PhysicsVector position{0,0,0};
    PhysicsVector velocity{0,0,0};
    float mCoefficientOfRestitution{.8};

private:

};

#endif // PHYSICSSPHERE_H
