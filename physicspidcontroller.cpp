#include "physicspidcontroller.h"

PhysicsPIDController::PhysicsPIDController(PhysicsRobot * robot)
{
    mRobot = robot;
    qGoal = Eigen::VectorXd::Zero(mRobot->numLinks);
    Kp = Eigen::MatrixXd::Identity(mRobot->numLinks,mRobot->numLinks)*0.0;
    Ki = Eigen::MatrixXd::Identity(mRobot->numLinks,mRobot->numLinks)*0.0;
    Kd = Eigen::MatrixXd::Identity(mRobot->numLinks,mRobot->numLinks)*0.0;
    integratedError = Eigen::VectorXd::Zero(mRobot->numLinks);
}

Eigen::VectorXd PhysicsPIDController::get_control_inputs(Eigen::VectorXd q, Eigen::VectorXd qd)
{
    if(check_antiwindup_criteria(q,qd))
    {
        integratedError = integratedError + (qGoal-q);
    }

    return Kp*(qGoal-q) + Ki*integratedError - Kd*qd;
}

bool PhysicsPIDController::check_antiwindup_criteria(Eigen::VectorXd q, Eigen::VectorXd qd)
{
    return true;
}
