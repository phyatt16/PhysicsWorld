#include "physicsrobot.h"
#include "physicsworld.h"

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
    linkCoMAccels.resize(numLinks);
    linkEndAccels.resize(numLinks);
    linkOmegas.resize(numLinks);
    linkAlphas.resize(numLinks);
    linkForces.resize(numLinks);
    linkTorques.resize(numLinks);
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
    calculate_robot_velocities_and_accelerations(q,qd,qdd);

    calculate_robot_forces_and_torques(q,qd,qdd,externalForces,externalTorques,g);

    return Eigen::VectorXd::Zero(3,1);
}

void PhysicsRobot::calculate_robot_velocities_and_accelerations(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd)
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

void PhysicsRobot::calculate_robot_forces_and_torques(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> externalForces, std::vector<Eigen::Vector3d> externalTorques, Eigen::Vector3d g)
{

    // F_i = F_i+1 + m_i * accel_i - m_i * g_i
    linkForces[numLinks-1] = joints[numLinks-1]->get_transform_from_parent_CoM_to_child_CoM().rotation()*externalForces[numLinks-1]
                  + joints[numLinks-1]->child->mass*linkCoMAccels[numLinks-1]
                  - joints[numLinks-1]->child->mass*(get_transform_from_base_to_link_CoM(numLinks-1).rotation()*g);

//    std::cout<<joints[numLinks-1]->child->mass*(get_transform_from_base_to_link_CoM(numLinks-1).rotation()*g)<<std::endl;
//    std::cout<<get_transform_from_base_to_link_CoM(numLinks-1).matrix()<<std::endl;
//    std::cout<<g<<std::endl;
//    std::cout<<get_transform_from_base_to_link_CoM(numLinks-1)*g<<std::endl;
//    std::cout<<joints[numLinks-1]->child->mass<<std::endl;

    // Tau_i = Tau_i+1 + F_i x -rc - F_i+1 x rc + I * alpha_i + omega_i x (I * omega_i)
    linkTorques[numLinks-1] = joints[numLinks-1]->get_transform_from_parent_CoM_to_child_CoM().rotation()*externalTorques[numLinks-1]
                   - linkForces[numLinks-1].cross(joints[numLinks-1]->TransformFromJointToChildCoM.translation())
                   + (joints[numLinks-1]->get_transform_from_parent_CoM_to_child_CoM().rotation()*externalForces[numLinks-1]).cross(-joints[numLinks-1]->TransformFromJointToChildCoM.translation() + joints[numLinks-1]->TransformFromJointToChildEnd.translation())
                   + joints[numLinks-1]->child->inertiaTensor*linkAlphas[numLinks-1]
                   + linkOmegas[numLinks-1].cross(joints[numLinks-1]->child->inertiaTensor*linkOmegas[numLinks-1]);

    //std::cout<<- linkForces[numLinks-1].cross(joints[numLinks-1]->TransformFromJointToChildCoM.translation())<<std::endl;
    //std::cout<<linkTorques[numLinks-1]<<std::endl;


    for(int i{numLinks-1}; i>=0; i=i-1)
    {
        // F_i = F_i+1 + m_i * accel_i - m_i * g_i
        linkForces[i] = joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation()*linkForces[i+1]
                      + joints[i]->child->mass*linkCoMAccels[i]
                      - joints[i]->child->mass*(get_transform_from_base_to_link_CoM(i).rotation()*g);

        // Tau_i = Tau_i+1 + F_i x -rc - F_i+1 x rc + I * alpha_i + omega_i x (I * omega_i)
        linkTorques[i] = joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation()*linkTorques[i+1]
                       - linkForces[i].cross(joints[i]->TransformFromJointToChildCoM.translation())
                       + (joints[i]->get_transform_from_parent_CoM_to_child_CoM().rotation()*linkForces[i+1]).cross(-joints[i]->TransformFromJointToChildCoM.translation() + joints[i]->TransformFromJointToChildEnd.translation())
                       + joints[i]->child->inertiaTensor*linkAlphas[i]
                       + linkOmegas[i].cross(joints[i]->child->inertiaTensor*linkOmegas[i]);
    }

}


PhysicsRobot create_n_link_robot(PhysicsWorld *world,int numLinks, double linkLengths)
{
    PhysicsRobot robot;

    PhysicsBox *box = new PhysicsBox;
    box->height = .1;
    box->length = .1;
    box->width = .1;
    robot.base = box;
    world->add_object_to_world(box);

    for(int i{0}; i<numLinks; i++)
    {
        PhysicsCylinder *cylinder = new PhysicsCylinder;
        cylinder->height = linkLengths;
        cylinder->radius = .1;
        cylinder->mass = 1;
        world->add_object_to_world(cylinder);
        PhysicsJoint * joint = new PhysicsJoint;
        if(i==0){joint->parent = robot.base;}
        else{joint->parent = robot.joints[i-1]->child;}

        joint->child = cylinder;
        joint->rotationAxis << 1, 0, 0;

        if(i==0){joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,0);}
        else{joint->TransformFromParentCoMToJoint = Eigen::Translation3d(0,0,cylinder->height/2.0);}

        joint->TransformFromJointToChildCoM = Eigen::Translation3d(0,0,cylinder->height/2.0);
        joint->TransformFromJointToChildEnd = Eigen::Translation3d(0,0,cylinder->height);
        robot.add_joint(joint);
    }

    return robot;
}
