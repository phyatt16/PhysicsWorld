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

TEST(GivenPhysicsVectors,WhenCalculatingDotProduct_DotProductIsCorrect)
{
    PhysicsVector vec1{-1.f,1.f,0.f};
    PhysicsVector vec2{1.f,-8.f,3.f};
    PhysicsVector vec3{0.f,0.f,3.f};

    float expectedDotProduct{0};

    expectedDotProduct = vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z;

    EXPECT_EQ(expectedDotProduct,vec1.dot(vec2));
    EXPECT_EQ(0,vec1.dot(vec3));

}



