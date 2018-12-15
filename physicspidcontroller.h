#ifndef PHYSICSPIDCONTROLLER_H
#define PHYSICSPIDCONTROLLER_H
#include "physicsrobot.h"


class PhysicsPIDController
{
public:
    PhysicsPIDController(PhysicsRobot * robot);
    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Ki;
    Eigen::MatrixXd Kd;
    Eigen::VectorXd qGoal;
    Eigen::VectorXd integratedError;

    Eigen::VectorXd get_control_inputs(Eigen::VectorXd q, Eigen::VectorXd qd);


private:
    PhysicsRobot * mRobot;

    bool check_antiwindup_criteria(Eigen::VectorXd q, Eigen::VectorXd qd);
};

#endif // PHYSICSPIDCONTROLLER_H
