#include "physicsworld.h"

#include <gtest/gtest.h>

TEST(PhysicsWorldTest,WhenInitialized_DefaultValuesAreCorrect)
{
    PhysicsWorld world;
    EXPECT_EQ(0,world.g(0));
    EXPECT_EQ(0,world.g(1));
    EXPECT_EQ(-9.81,world.g(2));
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

TEST(RoboticsUnitTest,WhenPuttingJointsBetweenLinks_TheyReturnRelativeTransformBetweenLinks)
{
    PhysicsWorld world;

    PhysicsCylinder *cylinder1 = new PhysicsCylinder;
    cylinder1->height = 1.5;
    cylinder1->radius = .1;
    world.add_object_to_world(cylinder1);

    PhysicsCylinder *cylinder2 = new PhysicsCylinder;
    cylinder2->height = 1.5;
    cylinder2->radius = .1;
    world.add_object_to_world(cylinder2);

    PhysicsJoint * joint1 = new PhysicsJoint;
    joint1->parent = cylinder1;
    joint1->child = cylinder2;
    joint1->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,1);
    joint1->rotationAxis << 0, 0, 1;
    joint1->TransformFromJointToChildCoM = Eigen::Translation3d(0,1,0) * Eigen::AngleAxisd(3.14159,Eigen::Vector3d(0,0,1));


    Eigen::Affine3d ExpectedTransformation = Eigen::Translation3d(0,1,1) * Eigen::AngleAxisd(3.14159,Eigen::Vector3d(0,0,1));

    Eigen::Affine3d actualTransformation = joint1->get_transform_from_parent_CoM_to_child_CoM();

    EXPECT_EQ(ExpectedTransformation(0,0),actualTransformation(0,0));
    EXPECT_EQ(ExpectedTransformation(1,0),actualTransformation(1,0));
    EXPECT_EQ(ExpectedTransformation(2,0),actualTransformation(2,0));
    EXPECT_EQ(ExpectedTransformation(3,0),actualTransformation(3,0));
    EXPECT_EQ(ExpectedTransformation(0,1),actualTransformation(0,1));
    EXPECT_EQ(ExpectedTransformation(1,1),actualTransformation(1,1));
    EXPECT_EQ(ExpectedTransformation(2,1),actualTransformation(2,1));
    EXPECT_EQ(ExpectedTransformation(3,1),actualTransformation(3,1));
    EXPECT_EQ(ExpectedTransformation(0,2),actualTransformation(0,2));
    EXPECT_EQ(ExpectedTransformation(1,2),actualTransformation(1,2));
    EXPECT_EQ(ExpectedTransformation(2,2),actualTransformation(2,2));
    EXPECT_EQ(ExpectedTransformation(3,2),actualTransformation(3,2));
    EXPECT_EQ(ExpectedTransformation(0,3),actualTransformation(0,3));
    EXPECT_EQ(ExpectedTransformation(1,3),actualTransformation(1,3));
    EXPECT_EQ(ExpectedTransformation(2,3),actualTransformation(2,3));
    EXPECT_EQ(ExpectedTransformation(3,3),actualTransformation(3,3));

}

TEST(RoboticsUnitTest,WhenCreatingRobotAndGivingJointAngles_RobotCanDoForwardKinematics)
{
    PhysicsWorld world;

    PhysicsRobot robot;

    PhysicsBox *box = new PhysicsBox;
    box->height = 1.0;
    box->length = 1.0;
    box->width = 1.0;

    robot->base = box;

    PhysicsCylinder *cylinder1 = new PhysicsCylinder;
    cylinder1->height = 1.0;
    cylinder1->radius = .1;
    world.add_object_to_world(cylinder1);

    PhysicsJoint * joint1 = new PhysicsJoint;
    joint1->parent = robot.base;
    joint1->child = cylinder1;
    joint1->TransformFromParentCoMToJoint = Eigen::Translation3d(0,robot->width/2.0,);
    joint1->rotationAxis << 1, 0, 0;
    joint1->TransformFromJointToChildCoM = Eigen::Translation3d(0,cylinder1->height/2.0,0) * Eigen::AngleAxisd(0,Eigen::Vector3d(0,0,1));

    robot.add_joint(joint1);

    PhysicsCylinder *cylinder2 = new PhysicsCylinder;
    cylinder2->height = 1.0;
    cylinder2->radius = .1;
    world.add_object_to_world(cylinder2);

    PhysicsJoint * joint2 = new PhysicsJoint;
    joint1->parent = cylinder1;
    joint1->child = cylinder2;
    joint1->TransformFromParentCoMToJoint = Eigen::Translation3d(0,cylinder1->height/2.0,0);
    joint1->rotationAxis << 1, 0, 0;
    joint1->TransformFromJointToChildCoM = Eigen::Translation3d(0,cylinder2->height/2.0,0) * Eigen::AngleAxisd(0,Eigen::Vector3d(0,0,1));

    Eigen::Affine3d robotEEPose = robot.get_pose_of_link_end(1);

}



