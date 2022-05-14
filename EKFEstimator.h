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
        void setSensorData(Vector3d acc, Vector3d gyro);

        void predict();
        void predictState();
        
        void update();
        void updateState();

    
    private:
        Vector3d GLeftFootPos_;
        Vector3d GRightFootPos_;
        Vector3d GBasePos_;
        Vector3d GBaseVel_;
        Quaterniond GBaseAtitude_;
        Matrix3d GBaseRot_;


        Vector3d BAcc_;
        Vector3d BGyro_;

        Vector3d BAccBias_;
        Vector3d BGyroBias_;

        VectorXd xPrior_;
        VectorXd xPosterior_;

        vector<double> Config_;
        vector<double> ConfigDot_;

        Vector3d Gravity_;

        int statesDim_;
        int rvqStatesDim_;
        double dt_;

};