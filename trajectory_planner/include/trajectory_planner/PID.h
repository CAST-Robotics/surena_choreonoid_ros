#pragma once

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"
#include <eigen3/Eigen/Eigen>
#include <trajectory_planner/DCMController.h>
#include <ros/ros.h>


class PID{
    public:
        PID(Matrix3d kp, Matrix3d ki, Matrix3d kd, double timeStep);

    private:
        ros::Nodehandle nh
        // Controller Gains 
        double kp_;
        double ki_;
        double kd_;


        double dt_;

        double prevoiusError_;   // Controller Previous Error
        double intI_;            // Controller Integrator

        double getOutput(double deiredValue, double currentValue);
};
