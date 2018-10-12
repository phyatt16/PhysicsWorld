#include "physicsvector.h"

PhysicsVector::PhysicsVector()
{
    x = 0;
    y = 0;
    z = 0;
}
PhysicsVector::PhysicsVector(float x_, float y_, float z_)
{
    x = x_;
    y = y_;
    z = z_;
}

void PhysicsVector::floor(float minValue)
{
    if(x<minValue){x=minValue;}
    if(y<minValue){y=minValue;}
    if(z<minValue){z=minValue;}
}

void PhysicsVector::ceil(float maxValue)
{
    if(x>maxValue){x=maxValue;}
    if(y>maxValue){y=maxValue;}
    if(z>maxValue){z=maxValue;}
}
