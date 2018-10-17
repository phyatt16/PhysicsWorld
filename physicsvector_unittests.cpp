#include "physicsvector.h"
#include <gtest/gtest.h>

TEST(GivenPhysicsVector,WhenCreatingVector_VectorInitializesToZeros)
{
    PhysicsVector vec;
    EXPECT_EQ(0,vec.x);
    EXPECT_EQ(0,vec.y);
    EXPECT_EQ(0,vec.z);
}

TEST(GivenPhysicsVector,WhenAddingVectors_VectorSumsCorrectly)
{
    PhysicsVector vec1{1,2,3};
    PhysicsVector vec2{4,5,6};

    PhysicsVector sumVector = vec1 + vec2;
    EXPECT_EQ(5,sumVector.x);
    EXPECT_EQ(7,sumVector.y);
    EXPECT_EQ(9,sumVector.z);
}

TEST(GivenPhysicsVector,WhenMultiplyingByFloat_VectorMultipliesCorrectly)
{
    PhysicsVector vec1{1,2,3};
    float k{.5};

    PhysicsVector resultVector = vec1*k;
    EXPECT_EQ(.5f,resultVector.x);
    EXPECT_EQ(1.f,resultVector.y);
    EXPECT_EQ(1.5f,resultVector.z);
}

TEST(GivenPhysicsVector,WhenUsingFloorFunction_FloorFunctionWorks)
{
    PhysicsVector vec{1,2,3};
    float k{2};

    vec.floor(k);
    EXPECT_EQ(2.f,vec.x);
    EXPECT_EQ(2.f,vec.y);
    EXPECT_EQ(3.f,vec.z);
}

TEST(GivenPhysicsVector,WhenUsingCeilFunction_CeilFunctionWorks)
{
    PhysicsVector vec{1,2,3};
    float k{2};

    vec.ceil(k);
    EXPECT_EQ(1.f,vec.x);
    EXPECT_EQ(2.f,vec.y);
    EXPECT_EQ(2.f,vec.z);
}

TEST(GivenPhysicsVector,WhenCalculatingNorm_NormIsCorrect)
{
    PhysicsVector vec1{-1.f,0.f,0.f};
    float goalNorm1{1.f};

    EXPECT_EQ(goalNorm1,vec1.norm());

    PhysicsVector vec2{-1.f,2.f,-2.f};
    float goalNorm2{3.f};

    EXPECT_EQ(goalNorm2,vec2.norm());

}



