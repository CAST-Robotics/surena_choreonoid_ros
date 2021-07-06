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
        PID(double timeStep);

    private:
        ros::Nodehandle nh;
        // Controller Gains 
        Matrix3d kp_;
        Matrix3d ki_;
        Matrix3d kd_;
        Matrix3d kcom_;
        Matrix3d kzmp_;
        Vector3d x_error_

        double dt_;

        double prevoiusError_;   // Controller Previous Error
        double intI_;            // Controller Integrator

        //double getOutput(double deiredValue, double currentValue);
        bool dcmController(trajectory_planner::DCMController::Request &req,
                           trajectory_planner::DCMController::Response &res);
        bool comController(trajectory_planner::COMController::Request &req,
                           trajectory_planner::COMController::Response &res);
};
