#ifndef PHYSICSROBOT_H
#define PHYSICSROBOT_H
#include "physicsworld.h"

class PhysicsRobot
{
public:
    PhysicsRobot();

    void add_joint(PhysicsJoint * joint);
    void update_robot_kinematics();
    Eigen::Affine3d get_transform_from_base_to_link_end(int link_id);
    Eigen::Affine3d get_transform_from_base_to_link_CoM(int link_id);
    void set_joint_angles(Eigen::VectorXd q);
    Eigen::VectorXd get_joint_torques_RNE(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, Eigen::MatrixXd externalWrenches);
    void calculate_robot_velocities_and_accelerations(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd);
    void calculate_robot_forces_and_torques(Eigen::VectorXd q, Eigen::VectorXd qd, Eigen::VectorXd qdd, Eigen::MatrixXd externalWrenches);


    PhysicsObject * base;
    std::vector<PhysicsJoint*> joints;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qdd;
    std::vector<Eigen::Vector3d> linkCoMAccels;
    std::vector<Eigen::Vector3d> linkEndAccels;
    std::vector<Eigen::Vector3d> linkOmegas;
    std::vector<Eigen::Vector3d> linkAlphas;
    std::vector<Eigen::Vector3d> linkForces;
    std::vector<Eigen::Vector3d> linkTorques;

    int numLinks{0};

};

PhysicsRobot create_n_link_robot(PhysicsWorld *world, int numLinks, double linkLengths=1.0);

#endif // PHYSICSROBOT_H
