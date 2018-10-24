#ifndef PHYSICSVECTOR_H
#define PHYSICSVECTOR_H

class PhysicsVector
{
public:
    PhysicsVector();
    PhysicsVector(float x_, float y_, float z_);
    void floor(float minValue);
    void ceil(float maxValue);
    float norm();
    float dot(PhysicsVector &vec);
    float x;
    float y;
    float z;


private:

};

inline PhysicsVector operator+(const PhysicsVector &v1, const PhysicsVector &v2)
{
    return PhysicsVector{v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

inline PhysicsVector operator-(const PhysicsVector &v1, const PhysicsVector &v2)
{
    return PhysicsVector{v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

inline PhysicsVector operator*(const PhysicsVector &v1, const float &k)
{
    return PhysicsVector{v1.x*k, v1.y*k, v1.z*k};
}

inline PhysicsVector operator*(const float &k, const PhysicsVector &v1)
{
    return PhysicsVector{v1.x*k, v1.y*k, v1.z*k};
}

inline PhysicsVector operator-(PhysicsVector &v)
{
    return PhysicsVector{-v.x,-v.y,-v.z};
}






#endif // PHYSICSVECTOR_H
