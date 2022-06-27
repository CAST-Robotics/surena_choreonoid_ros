#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include <ros/package.h>

#include "riccati_solver.h"
#include "ZMPPlanner.h"

using namespace std;
using namespace Eigen;

class PreviewTraj {
    public:
        PreviewTraj(double robot_height, int n);

        void setDt(double dt);
        void computeWeight();
        void computeTraj();

    private:
        double dt_;
        double g_;
        double robotHeight_;
        Matrix3d A_;
        Vector3d b_;
        RowVector3d c_;
        MatrixXd Q_;
        MatrixXd R_;
        int N_;

        Vector3d x0_;
        Vector3d* x_;
        Vector3d* y_;
        MatrixXd P_;
        MatrixXd Gl_;
        MatrixXd Gx_;
        VectorXd Gd_;

        Vector3d error_;
        ZMPPlanner* ZMPPlanner_;
};