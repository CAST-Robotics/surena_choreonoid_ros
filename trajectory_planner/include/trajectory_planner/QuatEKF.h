#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "utils.h"

using namespace std;
using namespace Eigen;

class QuatEKF {
    public:
        QuatEKF();
        ~QuatEKF();
        void initializeStates(Quaterniond q, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                              Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias);
        void concatStates(Quaterniond q, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                          Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias, VectorXd &output);
        void initializeCovariance(double quat_std, double vel_std, double pos_std,
                                  double contact_std, double gyro_std, double acc_std);
        inline void setDt(double dt){dt_ = dt;}
        void setSensorData(Vector3d acc, Vector3d gyro);
        void setMeasuredData(Vector3d r_kynematic, Vector3d l_kynematic);
        void setNoiseStd(double gyro_noise, double acc_noise, double gyro_bias_noise, 
                         double acc_bias_noise, double contact_noise, double measurement_noise);

        void seprateStates(VectorXd &x);

        void predict();
        void predictState();
        void updateQd();
        void predictCov();
        
        void update();
        void updateRk();
        void updateHk();

        void runFilter(Vector3d acc, Vector3d gyro, Vector3d rfmeasured, Vector3d lfmeasured);

    
    private:
        Vector3d GLeftFootPos_;
        Vector3d GRightFootPos_;
        Vector3d BLeftFootPos_;
        Vector3d BRightFootPos_;
        Vector3d GBasePos_;
        Vector3d GBaseVel_;
        Quaterniond GBaseQuat_;
        Matrix3d GBaseRot_;

        Vector3d BRFootMeasured_;
        Vector3d BLFootMeasured_;
        Matrix3d BRFootRotMeasured_;
        Matrix3d BLFootRotMeasured_;
        int contact_[2];

        Vector3d BAcc_;
        Vector3d BGyro_;

        Vector3d BAccBias_;
        Vector3d BGyroBias_;

        VectorXd x_;
        VectorXd xPrev_;
        VectorXd z_;
        VectorXd y_;
        VectorXd deltaX_;


        MatrixXd P_;

        MatrixXd Lc_;
        MatrixXd Qd_;
        MatrixXd Phi_;
        MatrixXd Qk_;

        MatrixXd Rk_;
        MatrixXd Hk_;
        MatrixXd Sk_;
        MatrixXd Kk_;
        
        double gyroNoiseStd_;
        double accNoiseStd_;
        double gyroBiasNoiseStd_;
        double accBiasNoiseStd_;
        double contactNoiseStd_;

        double measurementNoiseStd_;

        //initial states standard deviations
        double quatStd_;
        double velStd_;
        double posStd_;
        double contactStd_;
        double gyroStd_;
        double accStd_;

        vector<double> Config_;
        vector<double> ConfigDot_;

        Vector3d Gravity_;

        int statesDim_;
        int statesErrDim_;
        int processNoiseDim_;
        int qvrStatesDim_;

        int measurmentDim_;
        double dt_;
        bool updateEnabled_;

};