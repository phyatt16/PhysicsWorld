#include "physicsworld.h"

#include <gtest/gtest.h>

class PhysicsWorldTest : public ::testing::Test
{
protected:
    void SetUp()
    {
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

TEST_F(PhysicsWorldTest,WhenInitialized_DefaultValuesAreCorrect)
{
    EXPECT_EQ(0,world.g.x);
    EXPECT_EQ(0,world.g.y);
    EXPECT_EQ(-9.81f,world.g.z);
}

TEST_F(PhysicsWorldTest,WhenAddingObjects_WorldKeepsCountOfObjects)
{
    PhysicsSphere *sphere = new PhysicsSphere;
    world.add_object_to_world(sphere);

    EXPECT_EQ(2,world.get_number_of_objects());
}

TEST(PhysicsWorldUnitTest,WhenAddingDifferentObjects_TheyRetainTheirUniqueProperties)
{
    PhysicsWorld world;

    PhysicsSphere *sphere = new PhysicsSphere;
    sphere->radius = 2;
    PhysicsCylinder *cylinder = new PhysicsCylinder;
    cylinder->height = 1.5;
    cylinder->radius = .1;
    PhysicsBox *box = new PhysicsBox;
    box->length = 3;
    box->height = 4;
    box->width = 5;

    world.add_object_to_world(sphere);
    world.add_object_to_world(cylinder);
    world.add_object_to_world(box);

    PhysicsSphere* someObject1 = dynamic_cast<PhysicsSphere*>(world.mObjects[0]);

    EXPECT_EQ("sphere",someObject1->shape);
    EXPECT_EQ(2,someObject1->radius);

    PhysicsCylinder* someObject2 = dynamic_cast<PhysicsCylinder*>(world.mObjects[1]);

    EXPECT_EQ("cylinder",someObject2->shape);
    EXPECT_EQ(.1f,someObject2->radius);
    EXPECT_EQ(1.5,someObject2->height);

    PhysicsBox* someObject3 = dynamic_cast<PhysicsBox*>(world.mObjects[2]);

    EXPECT_EQ("box",someObject3->shape);
    EXPECT_EQ(3,someObject3->length);
    EXPECT_EQ(4,someObject3->height);
    EXPECT_EQ(5,someObject3->width);


}



