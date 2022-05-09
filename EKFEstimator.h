#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using namespace Eigen;

class EKFEstimator {
    public:
        EKFEstimator();
        ~EKFEstimator();
        void predict();
        void update();

    
    private:
        Vector3d GLeftFootPos_;
        Vector3d GRightFootPos_;

        Vector3d BAcc_;
        Vector3d BGyro_;

        Vector3d BAccBias_;
        Vector3d BGyroBias_;

        vector<double> Config_;
        vector<double> ConfigDot_;

        Vector3d Gravity;

};