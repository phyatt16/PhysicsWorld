#include "physicsbox.h"

PhysicsBox::PhysicsBox(double boxLength, double boxWidth, double boxHeight, double boxMass)
{
    this->shape = "box";
    length = boxLength;
    width = boxWidth;
    height = boxHeight;
    mass = boxMass;
    inertiaTensor(0,0) = mass*(pow(width,2) +pow(height,2))/12.0;
    inertiaTensor(1,1) = mass*(pow(length,2) +pow(height,2))/12.0;
    inertiaTensor(2,2) = mass*(pow(width,2) +pow(length,2))/12.0;;
}
