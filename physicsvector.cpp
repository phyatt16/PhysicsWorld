#include "physicsvector.h"
#include <math.h>

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

float PhysicsVector::norm()
{
    return sqrt(pow(x,2.f) + pow(y,2.f) + pow(z,2.f));
}

float PhysicsVector::dot(PhysicsVector &vec)
{
    return this->x*vec.x + this->y*vec.y + this->z*vec.z;
}
