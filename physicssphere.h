#ifndef PHYSICSSPHERE_H
#define PHYSICSSPHERE_H
#include "physicsvector.h"

class PhysicsSphere
{
public:
    PhysicsSphere();
    float mass{1.f};
    float radius{0.f};
    float dragCoefficient{0.25f};
    PhysicsVector position{0.f,0.f,0.f};
    PhysicsVector velocity{0.f,0.f,0.f};
    float mCoefficientOfRestitution{.8f};

private:

};

#endif // PHYSICSSPHERE_H
