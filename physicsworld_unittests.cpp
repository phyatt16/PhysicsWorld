#include "physicsworld.h"
#include "physicsrobot.h"

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

    double radius{2};
    double mass{1};
    PhysicsSphere *sphere = new PhysicsSphere(radius,mass);
    double height = 1.5;
    radius = .1;
    PhysicsCylinder *cylinder = new PhysicsCylinder(height, radius, mass);
    double length = 3;
    double width = 5;
    height = 4;
    PhysicsBox *box = new PhysicsBox(length,width,height,mass);

    world.add_object_to_world(sphere);
    world.add_object_to_world(cylinder);
    world.add_object_to_world(box);

    PhysicsSphere* someObject1 = dynamic_cast<PhysicsSphere*>(world.mObjects[0]);

    EXPECT_EQ("sphere",someObject1->shape);
    EXPECT_EQ(2,someObject1->radius);

    PhysicsCylinder* someObject2 = dynamic_cast<PhysicsCylinder*>(world.mObjects[1]);

    EXPECT_EQ("cylinder",someObject2->shape);
    EXPECT_EQ(.1,someObject2->radius);
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
    double height = 1.5;
    double radius = .1;
    double mass = 1;
    PhysicsCylinder *cylinder1 = new PhysicsCylinder(height, radius, mass);
    cylinder1->height = 1.5;
    cylinder1->radius = .1;
    world.add_object_to_world(cylinder1);

    PhysicsCylinder *cylinder2 = new PhysicsCylinder(height, radius, mass);
    world.add_object_to_world(cylinder2);

    PhysicsJoint * joint1 = new PhysicsJoint;
    joint1->parent = cylinder1;
    joint1->child = cylinder2;
    joint1->rotationAxis << 0, 0, 1;
    joint1->TransformFromJointToChildCoM = Eigen::Translation3d(0,cylinder2->height/2.0,0);
    joint1->TransformFromJointToChildEnd = Eigen::Translation3d(0,cylinder2->height,0);

    joint1->set_joint_angle(3.14159/2.0);

    Eigen::Affine3d ExpectedTransformation = Eigen::AngleAxisd(3.14159/2.0,Eigen::Vector3d(0,0,1))*Eigen::Translation3d(0,1.5,0);
    Eigen::Affine3d actualTransformation = joint1->get_transform_from_parent_end_to_child_end();

    EXPECT_EQ(ExpectedTransformation.matrix(),actualTransformation.matrix());
}

TEST(RoboticsUnitTest,WhenCreatingRobotAndGivingJointAngles_RobotCanDoForwardKinematicsToEndsOfLinks)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{2};
    robot = create_n_link_robot(&world, numLinks);

    robot->joints[0]->jointAngle = 3.14159/2.0;
    robot->joints[1]->jointAngle = 3.14159/2.0;

    Eigen::Affine3d actualLink1EndPose = robot->get_transform_from_base_to_link_end(0);
    Eigen::Affine3d actualRobotEEPose = robot->get_transform_from_base_to_link_end(1);

    Eigen::Affine3d expectedLink1EndPose = Eigen::AngleAxisd(robot->joints[0]->jointAngle,Eigen::Vector3d(1,0,0))*Eigen::Translation3d(0,0,1);
    Eigen::Affine3d expectedRobotEEPose = Eigen::AngleAxisd(robot->joints[1]->jointAngle,Eigen::Vector3d(1,0,0))*Eigen::Translation3d(0,0,1)*Eigen::AngleAxisd(robot->joints[0]->jointAngle,Eigen::Vector3d(1,0,0))*Eigen::Translation3d(0,0,1);

    ASSERT_TRUE(actualLink1EndPose.isApprox(expectedLink1EndPose,.001));
    ASSERT_TRUE(actualRobotEEPose.isApprox(expectedRobotEEPose,.001));

}

TEST(RoboticsUnitTest,WhenCallingUpdateRobotKinematics_LinkObjectsHaveCorrectPoses)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{3};
    robot = create_n_link_robot(&world, numLinks);

    robot->joints[0]->jointAngle = 3.14159/2.0;
    robot->joints[1]->jointAngle = 3.14159/2.0;
    robot->joints[2]->jointAngle = 3.14159/2.0;

    robot->update_robot_kinematics();

    Eigen::Affine3d actualLink1Pose = robot->joints[0]->child->pose;
    Eigen::Affine3d expectedLink1Pose = Eigen::Translation3d(0,-.5,0)*Eigen::AngleAxisd(3.14159/2.0,Eigen::Vector3d(1,0,0));

    Eigen::Affine3d actualLink2Pose = robot->joints[1]->child->pose;
    Eigen::Affine3d expectedLink2Pose = Eigen::Translation3d(0,-1,-.5)*Eigen::AngleAxisd(3.14159,Eigen::Vector3d(1,0,0));

    Eigen::Affine3d actualLink3Pose = robot->joints[2]->child->pose;
    Eigen::Affine3d expectedLink3Pose = Eigen::Translation3d(0,-.5,-1)*Eigen::AngleAxisd(3.14159*3.0/2,Eigen::Vector3d(1,0,0));

    ASSERT_TRUE(actualLink1Pose.isApprox(expectedLink1Pose,.001));
    ASSERT_TRUE(actualLink2Pose.isApprox(expectedLink2Pose,.001));
    ASSERT_TRUE(actualLink3Pose.isApprox(expectedLink3Pose,.001));
}

TEST(RoboticsUnitTest,WhenCalculatingLinkAccelerations_LinkAccelerationsAreCorrect)
{
    PhysicsWorld world;
    PhysicsRobot *robot;
    int numLinks{3};

    std::vector<Eigen::Vector3d> linkCoMAccels;
    std::vector<Eigen::Vector3d> linkEndAccels;
    std::vector<Eigen::Vector3d> linkOmegas;
    std::vector<Eigen::Vector3d> linkAlphas;
    std::vector<Eigen::Vector3d> linkForces;
    std::vector<Eigen::Vector3d> linkTorques;
    linkCoMAccels.resize(numLinks);
    linkEndAccels.resize(numLinks);
    linkOmegas.resize(numLinks);
    linkAlphas.resize(numLinks);
    linkForces.resize(numLinks);
    linkTorques.resize(numLinks);

    double linkLengths = .4;
    robot = create_n_link_robot(&world, numLinks, linkLengths);

    Eigen::VectorXd q(numLinks);
    Eigen::VectorXd qd(numLinks);
    Eigen::VectorXd qdd(numLinks);

    q << 3.14159/4.0,3.14159/4.0,3.14159/4.0;
    qd << 3.14159/6.0,-3.14159/4.0,3.14159/3.0;
    qdd << -3.14159/6.0,3.14159/3.0,3.14159/6.0;

    robot->calculate_robot_velocities_and_accelerations(q,qd,qdd,linkCoMAccels,linkEndAccels,linkOmegas,linkAlphas);

    Eigen::Vector3d link1Accel = linkCoMAccels[0];
    Eigen::Vector3d link2Accel = linkCoMAccels[1];
    Eigen::Vector3d link3Accel = linkCoMAccels[2];
    Eigen::Vector3d expectedLink1Accel;
    expectedLink1Accel << 0,.1047,-.0548;
    Eigen::Vector3d expectedLink2Accel;
    expectedLink2Accel << 0,-.0342,-.2393;
    Eigen::Vector3d expectedLink3Accel;
    expectedLink3Accel << 0,-.4866,-.2041;

    ASSERT_TRUE(link1Accel.isApprox(expectedLink1Accel,.001));
    ASSERT_TRUE(link2Accel.isApprox(expectedLink2Accel,.001));
    ASSERT_TRUE(link3Accel.isApprox(expectedLink3Accel,.001));
}

TEST(RoboticsUnitTest,WhenCalculatingLinkForces_LinkForcesAreCorrect)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{3};
    double linkLengths = 1;
    robot = create_n_link_robot(&world, numLinks, linkLengths);
    std::vector<Eigen::Vector3d> linkCoMAccels;
    std::vector<Eigen::Vector3d> linkEndAccels;
    std::vector<Eigen::Vector3d> linkOmegas;
    std::vector<Eigen::Vector3d> linkAlphas;
    std::vector<Eigen::Vector3d> linkForces;
    std::vector<Eigen::Vector3d> linkTorques;
    linkCoMAccels.resize(numLinks);
    linkEndAccels.resize(numLinks);
    linkOmegas.resize(numLinks);
    linkAlphas.resize(numLinks);
    linkForces.resize(numLinks);
    linkTorques.resize(numLinks);

    Eigen::VectorXd q(numLinks);
    Eigen::VectorXd qd(numLinks);
    Eigen::VectorXd qdd(numLinks);

    q << 3.14159/4.0,3.14159/4.0,3.14159/4.0;
    qd << 3.14159/6.0,-3.14159/4.0,3.14159/3.0;
    qdd << -3.14159/6.0,3.14159/3.0,3.14159/6.0;

    robot->calculate_robot_velocities_and_accelerations(q,qd,qdd,linkCoMAccels,linkEndAccels,linkOmegas,linkAlphas);

    std::vector<Eigen::Vector3d> externalForces;
    std::vector<Eigen::Vector3d> externalTorques;
    for(int i{0}; i<numLinks; i++)
    {
        externalForces.push_back(Eigen::Vector3d::Zero());
        externalTorques.push_back(Eigen::Vector3d::Zero());
    }

    robot->calculate_robot_forces_and_torques(q,qd,qdd,externalForces,externalTorques,world.g,linkCoMAccels,linkOmegas,linkAlphas,linkForces,linkTorques);

    Eigen::Vector3d link1Force = linkForces[0];
    Eigen::Vector3d link2Force = linkForces[1];
    Eigen::Vector3d link3Force = linkForces[2];
    Eigen::Vector3d expectedLink1Force;
    expectedLink1Force << 0,21.9449,18.9731;
    Eigen::Vector3d expectedLink2Force;
    expectedLink2Force << 0,19.0352,-1.8193;
    Eigen::Vector3d expectedLink3Force;
    expectedLink3Force << 0,5.7203,-7.4470;

    ASSERT_TRUE(link1Force.isApprox(expectedLink1Force,.001));
    ASSERT_TRUE(link2Force.isApprox(expectedLink2Force,.001));
    ASSERT_TRUE(link3Force.isApprox(expectedLink3Force,.001));
}

TEST(RoboticsUnitTest,WhenCalculatingGravityTorquesForPlanarRobot_JointTorquesAreCorrect)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{3};
    robot = create_n_link_robot(&world, numLinks);

    Eigen::VectorXd q(numLinks);
    Eigen::VectorXd qd(numLinks);
    Eigen::VectorXd qdd(numLinks);
    q << 3.14159/2.0,0,0;
    qd << 0,0,0;
    qdd << 0,0,0;

    std::vector<Eigen::Vector3d> externalForces;
    std::vector<Eigen::Vector3d> externalTorques;
    for(int i{0}; i<numLinks; i++)
    {
        externalForces.push_back(Eigen::Vector3d::Zero());
        externalTorques.push_back(Eigen::Vector3d::Zero());
    }

    Eigen::VectorXd jointTorques = robot->get_joint_torques_RNE(q,qd,qdd,externalForces,externalTorques,world.g);
    Eigen::VectorXd expectedJointTorques(numLinks);
    expectedJointTorques << -44.145,-19.62,-4.905;

    ASSERT_TRUE(jointTorques.isApprox(expectedJointTorques,.001));
}

TEST(RoboticsUnitTest,WhenCalculatingJointTorquesWithExternalForce_JointTorquesAreCorrect)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{3};
    robot = create_n_link_robot(&world, numLinks);

    Eigen::VectorXd q(numLinks);
    Eigen::VectorXd qd(numLinks);
    Eigen::VectorXd qdd(numLinks);
    q << 3.14159/2.0,0,0;
    qd << 0,0,0;
    qdd << 0,0,0;

    std::vector<Eigen::Vector3d> externalForces;
    std::vector<Eigen::Vector3d> externalTorques;
    for(int i{0}; i<numLinks; i++)
    {
        externalForces.push_back(Eigen::Vector3d::Zero());
        externalTorques.push_back(Eigen::Vector3d::Zero());
    }

    externalForces[0] << 0,0,0;
    externalForces[1] << 0,0,0;
    externalForces[2](2)=-1;

    Eigen::VectorXd jointTorques = robot->get_joint_torques_RNE(q,qd,qdd,externalForces,externalTorques,world.g);
    Eigen::VectorXd expectedJointTorques(numLinks);
    expectedJointTorques << -47.145,-21.62,-5.905;

    ASSERT_TRUE(jointTorques.isApprox(expectedJointTorques,.001));
}

TEST(RoboticsUnitTest,WhenCalculatingMassMatrix_MassMatrixIsCorrect)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{3};
    robot = create_n_link_robot(&world, numLinks);

    Eigen::VectorXd q(numLinks);
    q << 3.14159/4.0,3.14159/4.0,3.14159/4.0;

    Eigen::MatrixXd actualMassMatrix = robot->get_mass_matrix(q);

    Eigen::MatrixXd expectedMassMatrix(numLinks,numLinks);
    expectedMassMatrix.row(0) << 6.8359, 3.4394, .6894;
    expectedMassMatrix.row(1) << 3.4394, 2.3788, .6894;
    expectedMassMatrix.row(2) << 0.6894, .6894, .3358;

    ASSERT_TRUE(expectedMassMatrix.isApprox(actualMassMatrix,.001));
}

TEST(RoboticsUnitTest,WhenCalculatingAccelerations_AccelerationsAreCorrect)
{
    PhysicsWorld world;
    PhysicsRobot *robot;

    int numLinks{3};
    robot = create_n_link_robot(&world, numLinks);

    Eigen::VectorXd q(numLinks);
    Eigen::VectorXd qd(numLinks);
    Eigen::VectorXd tau(numLinks);
    q << 0,0,0;
    qd << 0,0,0;
    tau << 0,0,0;

    Eigen::VectorXd actualAcceleration = robot->get_accel(q,qd,tau,world.g);

    Eigen::VectorXd expectedAcceleration(numLinks);
    expectedAcceleration << 0, 0, 0;

    std::cout<<actualAcceleration<<std::endl;

    ASSERT_TRUE(expectedAcceleration.isApprox(actualAcceleration,.001));
}







