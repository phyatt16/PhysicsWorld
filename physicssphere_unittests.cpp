#include "physicssphere.h"
#include <gtest/gtest.h>

TEST(GivenPhysicsSphere,WhenCreatingSphere_SphereHasZeroMass)
{
    PhysicsSphere sphere;
    EXPECT_EQ(0,sphere.mass);
}
