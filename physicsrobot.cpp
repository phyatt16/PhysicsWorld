#include "physicsrobot.h"
#include "physicsworld.h"
#include "eigen3/Eigen/LU"
#include <thread>

PhysicsRobot::PhysicsRobot()
{

}

void PhysicsRobot::add_joint(PhysicsJoint * joint)
{
    joints.push_back(joint);
    numLinks++;
    q.resize(numLinks,1);
    qd.resize(numLinks,1);
    qdd.resize(numLinks,1);
}

void PhysicsRobot::update_robot_kinematics()
{
    for(int i{0}; i<numLinks; i++)
    {
        this->joints[i]->update_joint_kinematics();
        this->joints[i]->child->pose = this->joints[i]->parent->pose*this->joints[i]->get_transform_from_parent_CoM_to_child_CoM();
    }
}

void PhysicsRobot::set_joint_angles(Eigen::VectorXd q)
{
    for(int i{0}; i<numLinks; i++)
    {
        this->joints[i]->jointAngle = q(i);
    }
}

Eigen::VectorXd PhysicsRobot::get_accel(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd tau, Eigen::Vector3d g)
{
    return get_mass_matrix(q).inverse() * (tau - get_coriolis_torques(q,qd) - get_gravity_torques(q,g));
}

void PhysicsRobot::calculate_column_of_mass_matrix(Eigen::VectorXd *q, Eigen::VectorXd *qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> *externalForces, std::vector<Eigen::Vector3d> *externalTorques, Eigen::Vector3d *g, Eigen::MatrixXd * M, int i)
{
    M->col(i) = get_joint_torques_RNE(*q,*qd,qdd,*externalForces,*externalTorques,*g);
}

Eigen::MatrixXd PhysicsRobot::get_mass_matrix(Eigen::VectorXd q)
{
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(numLinks);
    Eigen::VectorXd qdd = Eigen::VectorXd::Zero(numLinks);
    Eigen::Vector3d g = Eigen::VectorXd::Zero(3);

    std::vector<Eigen::Vector3d> externalForces;
    std::vector<Eigen::Vector3d> externalTorques;
    for(int i{0}; i<numLinks; i++)
    {
        externalForces.push_back(Eigen::Vector3d::Zero());
        externalTorques.push_back(Eigen::Vector3d::Zero());
    }

    Eigen::MatrixXd M(numLinks,numLinks);
    //std::vector<std::thread> threads;
    for(int i{0}; i<numLinks; i++)
    {
        qdd(i) = 1;
        //threads.push_back(std::thread(&PhysicsRobot::calculate_column_of_mass_matrix,this,&q,&qd,qdd,&externalForces,&externalTorques,&g,&M,i));
        M.col(i) = get_joint_torques_RNE(q,qd,qdd,externalForces,externalTorques,g);
        qdd(i) = 0;
    }

//    for(int i{0}; i<numLinks; i++)
//    {
//        threads[i].join();
//    }


    return M;
}

Eigen::VectorXd PhysicsRobot::get_coriolis_torques(Eigen::VectorXd q,Eigen::VectorXd qd)
{
    Eigen::VectorXd qdd = Eigen::VectorXd::Zero(numLinks);
    Eigen::Vector3d g = Eigen::VectorXd::Zero(3);

    std::vector<Eigen::Vector3d> externalForces;
    std::vector<Eigen::Vector3d> externalTorques;
    for(int i{0}; i<numLinks; i++)
    {
        externalForces.push_back(Eigen::Vector3d::Zero());
        externalTorques.push_back(Eigen::Vector3d::Zero());
    }

    Eigen::VectorXd coriolisTorques(numLinks);
    coriolisTorques = get_joint_torques_RNE(q,qd,qdd,externalForces,externalTorques,g);

    return coriolisTorques;
}

Eigen::VectorXd PhysicsRobot::get_gravity_torques(Eigen::VectorXd q, Eigen::Vector3d g)
{
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(numLinks);
    Eigen::VectorXd qdd = Eigen::VectorXd::Zero(numLinks);

    std::vector<Eigen::Vector3d> externalForces;
    std::vector<Eigen::Vector3d> externalTorques;
    for(int i{0}; i<numLinks; i++)
    {
        externalForces.push_back(Eigen::Vector3d::Zero());
        externalTorques.push_back(Eigen::Vector3d::Zero());
    }

    Eigen::VectorXd gravityTorques(numLinks);
    gravityTorques = get_joint_torques_RNE(q,qd,qdd,externalForces,externalTorques,g);

    return gravityTorques;
}


Eigen::Affine3d PhysicsRobot::get_transform_from_base_to_link_end(int link_id)
{
    update_robot_kinematics();
    return this->joints[link_id]->parent->pose*this->joints[link_id]->get_transform_from_parent_CoM_to_child_end();
}

Eigen::Affine3d PhysicsRobot::get_transform_from_base_to_link_CoM(int link_id)
{
    update_robot_kinematics();
    return this->joints[link_id]->parent->pose*this->joints[link_id]->get_transform_from_parent_CoM_to_child_CoM();
}

Eigen::VectorXd PhysicsRobot::get_joint_torques_RNE(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> externalForces, std::vector<Eigen::Vector3d> externalTorques, Eigen::Vector3d g)
{
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

    calculate_robot_velocities_and_accelerations(q,qd,qdd,linkCoMAccels,linkEndAccels,linkOmegas,linkAlphas);

    calculate_robot_forces_and_torques(q,qd,qdd,externalForces,externalTorques,g,linkCoMAccels,linkOmegas,linkAlphas,linkForces,linkTorques);

    Eigen::VectorXd jointTorques(numLinks);
    for(int i{0}; i<numLinks; i++)
    {
        jointTorques(i) = joints[i]->rotationAxis.dot(linkTorques[i]);
    }

    return jointTorques;
}

void PhysicsRobot::calculate_robot_velocities_and_accelerations(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> &linkCoMAccels, std::vector<Eigen::Vector3d> &linkEndAccels, std::vector<Eigen::Vector3d> &linkOmegas, std::vector<Eigen::Vector3d> &linkAlphas)
{
    set_joint_angles(q);

    linkOmegas[0] = joints[0]->rotationAxis*qd(0);
    linkAlphas[0] = joints[0]->rotationAxis*qdd(0)
                  + linkOmegas[0].cross(joints[0]->rotationAxis*qd(0));
    linkCoMAccels[0] = linkAlphas[0].cross(joints[0]->TransformFromJointToChildCoM.translation())
                     + linkOmegas[0].cross(linkOmegas[0].cross(joints[0]->TransformFromJointToChildCoM.translation()));
    linkEndAccels[0] = linkAlphas[0].cross(joints[0]->TransformFromJointToChildEnd.translation())
                     + linkOmegas[0].cross(linkOmegas[0].cross(joints[0]->TransformFromJointToChildEnd.translation()));

    for(int i{1}; i<numLinks; i++)
    {
        // Omega_link_i = omega_link_i-1 + omega_from_joint_velocity
        linkOmegas[i] = joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation().transpose()*linkOmegas[i-1]
                      + joints[i]->rotationAxis*qd(i);

        // Alpha_link_i = Alpha_link_i-1 + alpha_from_joint_acceleration + omega x qd_i
        linkAlphas[i] = joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation().transpose()*linkAlphas[i-1]
                      + joints[i]->rotationAxis*qdd(i)
                      + linkOmegas[i].cross(joints[i]->rotationAxis*qd(i));

        // ac_link_i = ae_link_i-1 + alpha_i x rc + omega_i x (omega_i x rc)
        linkCoMAccels[i] = joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation().transpose()*linkEndAccels[i-1]
                         + linkAlphas[i].cross(joints[i]->TransformFromJointToChildCoM.translation())
                         + linkOmegas[i].cross(linkOmegas[i].cross(joints[i]->TransformFromJointToChildCoM.translation()));


        // ae_link_i = ae_link_i-1 + alpha_i x re + omega_i x (omega_i x re)
        linkEndAccels[i] = joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation().transpose()*linkEndAccels[i-1]
                         + linkAlphas[i].cross(joints[i]->TransformFromJointToChildEnd.translation())
                         + linkOmegas[i].cross(linkOmegas[i].cross(joints[i]->TransformFromJointToChildEnd.translation()));

    }

}

void PhysicsRobot::calculate_robot_forces_and_torques(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> externalForces, std::vector<Eigen::Vector3d> externalTorques, Eigen::Vector3d g, std::vector<Eigen::Vector3d> &linkCoMAccels, std::vector<Eigen::Vector3d> &linkOmegas, std::vector<Eigen::Vector3d> &linkAlphas, std::vector<Eigen::Vector3d> &linkForces, std::vector<Eigen::Vector3d> &linkTorques)
{

    // F_i = F_i+1 + m_i * accel_i - m_i * g_i
    linkForces[numLinks-1] = -get_transform_from_base_to_link_CoM(numLinks-1).rotation().transpose()*externalForces[numLinks-1]
                           + joints[numLinks-1]->child->mass*linkCoMAccels[numLinks-1]
                           - joints[numLinks-1]->child->mass*(get_transform_from_base_to_link_CoM(numLinks-1).rotation().transpose()*g);

    // Tau_i = Tau_i+1 + F_i x -rc - F_i+1 x rc + I * alpha_i + omega_i x (I * omega_i)
    linkTorques[numLinks-1] = -get_transform_from_base_to_link_CoM(numLinks-1).rotation().transpose()*externalTorques[numLinks-1]
                   - linkForces[numLinks-1].cross(joints[numLinks-1]->TransformFromJointToChildCoM.translation())
                   - (get_transform_from_base_to_link_CoM(numLinks-1).rotation().transpose()*externalForces[numLinks-1]).cross(joints[numLinks-1]->TransformFromJointToChildCoM.translation() - joints[numLinks-1]->TransformFromJointToChildEnd.translation())
                   + joints[numLinks-1]->child->inertiaTensor*linkAlphas[numLinks-1]
                   + linkOmegas[numLinks-1].cross(joints[numLinks-1]->child->inertiaTensor*linkOmegas[numLinks-1]);



    for(int i{numLinks-2}; i>=0; i--)
    {
        // F_i = F_i+1 + m_i * accel_i - m_i * g_i
        linkForces[i] = -get_transform_from_base_to_link_CoM(i).rotation().transpose()*externalForces[i]
                      + joints[i+1]->get_transform_from_parent_CoM_to_child_CoM().rotation()*linkForces[i+1]
                      + joints[i]->child->mass*linkCoMAccels[i]
                      - joints[i]->child->mass*(get_transform_from_base_to_link_CoM(i).rotation().transpose()*g);

        // Tau_i = Tau_i+1 + F_i x -rc - F_i+1 x rc + I * alpha_i + omega_i x (I * omega_i)
        linkTorques[i] = -get_transform_from_base_to_link_CoM(i).rotation().transpose()*externalTorques[i]
                       + joints[i+1]->get_transform_from_parent_CoM_to_child_CoM().rotation()*linkTorques[i+1]
                       - linkForces[i].cross(joints[i]->TransformFromJointToChildCoM.translation())
                       - (get_transform_from_base_to_link_CoM(i).rotation().transpose()*externalForces[i]).cross(joints[i]->TransformFromJointToChildCoM.translation() - joints[i]->TransformFromJointToChildEnd.translation())
                       + (joints[i+1]->get_transform_from_parent_CoM_to_child_CoM().rotation()*linkForces[i+1]).cross(joints[i]->TransformFromJointToChildCoM.translation() - joints[i]->TransformFromJointToChildEnd.translation())
                       + joints[i]->child->inertiaTensor*linkAlphas[i]
                       + linkOmegas[i].cross(joints[i]->child->inertiaTensor*linkOmegas[i]);
    }

}


PhysicsRobot* create_n_link_robot(PhysicsWorld *world,int numLinks, double linkLengths, std::string shape)
{
    PhysicsRobot * robot = new PhysicsRobot;

    double length{.1};
    double height{.1};
    double width{.1};
    double mass{10};
    PhysicsBox *box = new PhysicsBox(length,height,width,mass);
    robot->base = box;
    world->add_object_to_world(box);

    if(shape=="cylinder")
    {
        for(int i{0}; i<numLinks; i++)
        {
            double height = linkLengths;
            double radius = .1;
            PhysicsCylinder *cylinder = new PhysicsCylinder(height,radius,mass);
            world->add_object_to_world(cylinder);
            PhysicsJoint * joint = new PhysicsJoint;
            if(i==0){joint->parent = robot->base;}
            else{joint->parent = robot->joints[i-1]->child;}

            joint->child = cylinder;
            joint->rotationAxis << 0, 1, 0;

            if(i==0){joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,0);}
            else{joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,cylinder->height/2.0);}

            joint->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,cylinder->height/2.0);
            joint->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,cylinder->height);
            robot->add_joint(joint);
        }
    }
    if(shape=="sphere")
    {
        for(int i{0}; i<numLinks; i++)
        {
            double radius = linkLengths;
            PhysicsSphere *link = new PhysicsSphere(radius,mass);
            world->add_object_to_world(link);
            PhysicsJoint * joint = new PhysicsJoint;
            if(i==0){joint->parent = robot->base;}
            else{joint->parent = robot->joints[i-1]->child;}

            joint->child = link;
            joint->rotationAxis << 0, 1, 0;

            if(i==0){joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,0);}
            else{joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,link->radius);}

            joint->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link->radius);
            joint->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,link->radius*2.0);
            robot->add_joint(joint);
        }
    }
    if(shape=="box")
    {
        for(int i{0}; i<numLinks; i++)
        {
            double height = linkLengths;
            double length = .2;
            double width = .2;

            PhysicsBox *link = new PhysicsBox(length,width,height,mass);
            world->add_object_to_world(link);
            PhysicsJoint * joint = new PhysicsJoint;
            if(i==0){joint->parent = robot->base;}
            else{joint->parent = robot->joints[i-1]->child;}

            joint->child = link;
            joint->rotationAxis << 0, 1, 0;

            if(i==0){joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,0);}
            else{joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,link->height/2.0);}

            joint->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link->height/2.0);
            joint->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,link->height);
            robot->add_joint(joint);
        }
    }



    return robot;
}

PhysicsRobot* create_baxter_robot(PhysicsWorld *world)
{
    PhysicsRobot * robot = new PhysicsRobot;

    double height = .10;
    double radius = .075;
    double mass = 1;
    PhysicsCylinder *link0 = new PhysicsCylinder(height,radius,mass);
    robot->base = link0;
    world->add_object_to_world(link0);

    // Joint 1
    height = .27035;
    radius = .08;
    mass = 1;
    PhysicsCylinder *link1 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link1);
    PhysicsJoint * joint1 = new PhysicsJoint;
    joint1->parent = robot->base;
    joint1->child = link1;
    joint1->rotationAxis << 0, 0, 1;
    joint1->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,.05);
    joint1->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link1->height/2.0);
    joint1->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,link1->height);
    robot->add_joint(joint1);

    // Joint 2
    height = .001;
    radius = .08;
    mass = 1;
    PhysicsCylinder *link2 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link2);
    PhysicsJoint * joint2 = new PhysicsJoint;
    joint2->parent = joint1->child;
    joint2->child = link2;
    joint2->rotationAxis << 0, 1, 0;
    joint2->TransformFromParentCoMToJoint = joint1->TransformFromJointToChildCoM.inverse()*joint1->TransformFromJointToChildEnd*Eigen::AngleAxisd(3.14159/2.0,Eigen::Vector3d(0,1,0));
    joint2->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link2->height/2.0);
    joint2->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,link2->height);
    robot->add_joint(joint2);

    // Joint 3
    height = .36435;
    radius = .08;
    mass = 1;
    PhysicsCylinder *link3 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link3);
    PhysicsJoint * joint3 = new PhysicsJoint;
    joint3->parent = joint2->child;
    joint3->child = link3;
    joint3->rotationAxis << 0, 0, 1;
    joint3->TransformFromParentCoMToJoint = joint2->TransformFromJointToChildCoM.inverse()*joint2->TransformFromJointToChildEnd;
    joint3->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link3->height/2.0);
    joint3->TransformFromJointToChildEnd = Eigen::Translation3d(0.069,0,link3->height);
    robot->add_joint(joint3);


    // Joint 4
    height = .001;
    radius = .06;
    mass = 1;
    PhysicsCylinder *link4 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link4);
    PhysicsJoint * joint4 = new PhysicsJoint;
    joint4->parent = joint3->child;
    joint4->child = link4;
    joint4->rotationAxis << 0, 1, 0;
    joint4->TransformFromParentCoMToJoint = joint3->TransformFromJointToChildCoM.inverse()*joint3->TransformFromJointToChildEnd;
    joint4->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link4->height/2.0);
    joint4->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,link4->height);
    robot->add_joint(joint4);


    // Joint 5
    height = .37429;
    radius = .06;
    mass = 1;
    PhysicsCylinder *link5 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link5);
    PhysicsJoint * joint5 = new PhysicsJoint;
    joint5->parent = joint4->child;
    joint5->child = link5;
    joint5->rotationAxis << 0, 0, 1;
    joint5->TransformFromParentCoMToJoint = joint4->TransformFromJointToChildCoM.inverse()*joint4->TransformFromJointToChildEnd;
    joint5->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link5->height/2.0);
    joint5->TransformFromJointToChildEnd = Eigen::Translation3d(.01,0,link5->height);
    robot->add_joint(joint5);

    // Joint 6
    height = .001;
    radius = .04;
    mass = 1;
    PhysicsCylinder *link6 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link6);
    PhysicsJoint * joint6 = new PhysicsJoint;
    joint6->parent = joint5->child;
    joint6->child = link6;
    joint6->rotationAxis << 0, 1, 0;
    joint6->TransformFromParentCoMToJoint = joint5->TransformFromJointToChildCoM.inverse()*joint5->TransformFromJointToChildEnd;
    joint6->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link6->height/2.0);
    joint6->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,link6->height);
    robot->add_joint(joint6);


    // Joint 7
    height = .229525;
    radius = .04;
    mass = 1;
    PhysicsCylinder *link7 = new PhysicsCylinder(height,radius,mass);
    world->add_object_to_world(link7);
    PhysicsJoint * joint7 = new PhysicsJoint;
    joint7->parent = joint6->child;
    joint7->child = link7;
    joint7->rotationAxis << 0, 0, 1;
    joint7->TransformFromParentCoMToJoint = joint6->TransformFromJointToChildCoM.inverse()*joint6->TransformFromJointToChildEnd;
    joint7->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,link7->height/2.0);
    joint7->TransformFromJointToChildEnd = Eigen::Translation3d(.01,0,link7->height);
    robot->add_joint(joint7);

    return robot;
}
