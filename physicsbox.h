#ifndef PHYSICSBOX_H
#define PHYSICSBOX_H
#include "physicsobject.h"

class PhysicsBox : public PhysicsObject
{
public:
    PhysicsBox();
    ~PhysicsBox(){};
    float length;
    float width;
    float height;
};

#endif // PHYSICSBOX_H
