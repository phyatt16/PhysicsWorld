#ifndef PHYSICSROBOT_H
#define PHYSICSROBOT_H
#include "physicssphere.h"
#include "physicscylinder.h"
#include "physicsbox.h"
#include "physicsjoint.h"
#include "physicsobject.h"
#include <vector>
//#include "physicsworld.h"

class PhysicsWorld;


class PhysicsRobot
{
public:
    PhysicsRobot();

    void add_joint(PhysicsJoint * joint);
    void update_robot_kinematics();
    Eigen::Affine3d get_transform_from_base_to_link_end(int link_id);
    Eigen::Affine3d get_transform_from_base_to_link_CoM(int link_id);
    void set_joint_angles(Eigen::VectorXd q);

    void calculate_column_of_mass_matrix(Eigen::VectorXd *q, Eigen::VectorXd *qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> *externalForces, std::vector<Eigen::Vector3d> *externalTorques, Eigen::Vector3d *g, Eigen::MatrixXd *M, int col);
    Eigen::MatrixXd get_mass_matrix(Eigen::VectorXd q);
    Eigen::VectorXd get_accel(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd tau, Eigen::Vector3d g);
    Eigen::VectorXd get_coriolis_torques(Eigen::VectorXd q, Eigen::VectorXd qd);
    Eigen::VectorXd get_gravity_torques(Eigen::VectorXd q, Eigen::Vector3d g);

    Eigen::VectorXd get_joint_torques_RNE(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> externalForces, std::vector<Eigen::Vector3d> externalTorques, Eigen::Vector3d g);
    void calculate_robot_velocities_and_accelerations(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> &linkCoMAccels, std::vector<Eigen::Vector3d> &linkEndAccels, std::vector<Eigen::Vector3d> &linkOmegas, std::vector<Eigen::Vector3d> &linkAlphas);
    void calculate_robot_forces_and_torques(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, std::vector<Eigen::Vector3d> externalForces, std::vector<Eigen::Vector3d> externalTorques, Eigen::Vector3d g,std::vector<Eigen::Vector3d> &linkCoMAccels, std::vector<Eigen::Vector3d> &linkOmegas, std::vector<Eigen::Vector3d> &linkAlphas, std::vector<Eigen::Vector3d> &linkForces, std::vector<Eigen::Vector3d> &linkTorques);


    PhysicsObject * base;
    std::vector<PhysicsJoint*> joints;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;


    int numLinks{0};

};

PhysicsRobot* create_n_link_robot(PhysicsWorld *world, int numLinks, double linkLengths=1.0);
PhysicsRobot* create_baxter_robot(PhysicsWorld *world);

#endif // PHYSICSROBOT_H
