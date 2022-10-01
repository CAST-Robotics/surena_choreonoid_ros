#pragma once

#include"Eigen/Dense"
#include "Eigen/Core"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"
#include "yaml-cpp/yaml.h"

using namespace Eigen;
using namespace std;

class MinJerk{
    public:
        MinJerk(){}
        MinJerk(bool use_file, double dt=0.005);
        ~MinJerk();
        void setConfigPath(string config_path);
        void parseConfig(YAML::Node config);
        int getTrajSize();
        void cubicPolyTraj(const MatrixXd& way_points, const VectorXd& time_points, double dt, const MatrixXd& vel_points, MatrixXd& q);
        Vector4d genCubicCoeffs(const double pos_pts[], const double vel_pts[], double final_time);

    protected:
        double dt_;
        bool useFile_;
        YAML::Node config_;
        double initDSPDuration_;
        double DSPDuration_;
        double SSPDuration_;
        double finalDSPDuration_;
        double stepHeight_;
        vector<Vector3d> footSteps_;
        int footStepCount_;
        int trajSize_;
        bool leftFirst_;

        template <typename T>
        T* cubicInterpolate(T theta_ini, T theta_f, T theta_dot_ini, T theta_dot_f, double tf){
            /* 
                Returns Cubic Polynomial with the Given Boundary Conditions
                https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
            */
            T* coefs = new T[4]; // a0, a1, a2, a3
            coefs[0] = theta_ini;
            coefs[1] = theta_dot_ini;
            coefs[2] = 3/pow(tf,2) * (theta_f - theta_ini) - 1/tf * (2 * theta_dot_ini + theta_dot_f);
            coefs[3] = -2/pow(tf,3) * (theta_f - theta_ini) + 1/pow(tf,2) * (theta_dot_ini + theta_dot_f);
            return coefs;
        }

        Vector3d* poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf);
        Vector3d* poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf, Vector3d x_d_ini, Vector3d _d_f, Vector3d x_dd_ini, Vector3d x_dd_f);
        Vector3d* ankle5Poly(Vector3d x0, Vector3d xf, double z_max, double tf);
        Vector3d* minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf);
        void write2File(Vector3d* input, int size, string file_name);
};