#include "physicsworld.h"
#include "physicssphere.h"
#include <gtest/gtest.h>

class PhysicsWorldTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        PhysicsVector gravityVector{0,0,0};
        world.g = gravityVector;

        PhysicsSphere *sphere = new PhysicsSphere;
        PhysicsVector sphereVelocity{1,0,0};
        PhysicsVector spherePosition{1,2,3};
        sphere->radius = 1;
        sphere->mass = 1;
        sphere->velocity = sphereVelocity;
        sphere->position = spherePosition;

        world.add_object_to_world(sphere);
    }
    PhysicsWorld world;
};

class PhysicsWorldWithSpheresAndGravityTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        PhysicsVector gravityVector{0,0,-9.81};
        world.g = gravityVector;

        PhysicsSphere *sphere = new PhysicsSphere;
        PhysicsVector sphereVelocity{1,1,1};
        PhysicsVector spherePosition{1,2,3};
        sphere->radius = 1;
        sphere->mass = 1;
        sphere->velocity = sphereVelocity;
        sphere->position = spherePosition;
        world.add_object_to_world(sphere);

        PhysicsSphere *sphere2 = new PhysicsSphere;
        PhysicsVector sphere2Velocity{-1,-1,-1};
        PhysicsVector sphere2Position{.5,2,3};
        sphere2->radius = 1;
        sphere2->mass = 1;
        sphere2->velocity = sphere2Velocity;
        sphere2->position = sphere2Position;
        world.add_object_to_world(sphere2);
    }
    PhysicsWorld world;
};

TEST_F(PhysicsWorldTest,WhenInitialized_DefaultValuesAreCorrect)
{
    EXPECT_EQ(0,world.g.x);
    EXPECT_EQ(0,world.g.y);
    EXPECT_EQ(0,world.g.z);
}

TEST_F(PhysicsWorldTest,WhenAddingObjects_WorldKeepsCountOfObjects)
{
    PhysicsSphere *sphere = new PhysicsSphere;
    world.add_object_to_world(sphere);

    EXPECT_EQ(2,world.get_number_of_objects());
}

TEST_F(PhysicsWorldTest,WhenSimulatingOneTimeStepWithGravity_SphereHasCorrectState)
{
    float timestep = .01;
    PhysicsSphere * sphereOne = world.get_object(0);
    world.g.z = -9.81;
    PhysicsVector sphere1ExpectedPosition;
    PhysicsVector sphere1ExpectedVelocity;

    sphere1ExpectedVelocity = sphereOne->velocity + world.g*timestep;
    sphere1ExpectedPosition = sphereOne->position + sphere1ExpectedVelocity*timestep;

    world.simulate_one_timestep(timestep);

    EXPECT_EQ(sphere1ExpectedPosition.x,sphereOne->position.x);
    EXPECT_EQ(sphere1ExpectedPosition.y,sphereOne->position.y);
    EXPECT_EQ(sphere1ExpectedPosition.z,sphereOne->position.z);
    EXPECT_EQ(sphere1ExpectedVelocity.x,sphereOne->velocity.x);
    EXPECT_EQ(sphere1ExpectedVelocity.y,sphereOne->velocity.y);
    EXPECT_EQ(sphere1ExpectedVelocity.z,sphereOne->velocity.z);
}


TEST_F(PhysicsWorldWithSpheresAndGravityTest,WhenMultipleSpheresAreInitializedInCollision_SpheresAreMovedToNotBeInCollision)
{
    PhysicsVector sphere1Position{world.get_object(0)->position};
    PhysicsVector sphere2Position{world.get_object(1)->position};
    float sumOfRadii{world.get_object(0)->radius + world.get_object(1)->radius};

    PhysicsVector fromSphere1ToSphere2{sphere1Position-sphere2Position};
    float distanceBetweenSpheres{fromSphere1ToSphere2.norm()};
    EXPECT_GE(distanceBetweenSpheres,sumOfRadii);
}
