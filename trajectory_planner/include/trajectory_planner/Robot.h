#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"

#include "DCM.h"
#include "Link.h"
#include "PID.h"
#include "Ankle.h"
#include "MinJerk.h"

#include "fstream"

using namespace std;

class Robot{
    friend class Surena;
    public:
        Robot(ros::NodeHandle *nh);
        ~Robot();

        void spinOnline(int iter, double config[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer);
        void spinOffline(int iter, double* config);
        bool jntAngsCallback(trajectory_planner::JntAngs::Request  &req,
                            trajectory_planner::JntAngs::Response &res);
        bool trajGenCallback(trajectory_planner::Trajectory::Request  &req,
                            trajectory_planner::Trajectory::Response &res);
    private:

        double thigh_;
        double shank_;
        double torso_;


        double joints_[12];

        PID* DCMController_;
        PID* CoMController_;

        void doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR);
        double* geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft);

        Matrix3d Rroll(double phi);
        Matrix3d RPitch(double theta);

        Vector3d* comd_;
        Vector3d* zmpd_;
        Vector3d* zmpr_;
        Vector3d* rAnkle_;
        Vector3d* lAnkle_;

        Vector3d rSole_;    // current position of right sole
        Vector3d lSole_;    // current position of left sole
        Vector3d* FKCoM_;      // current CoM of robot
        Vector3d* realZMP_;      // current ZMP of robot
        Vector3d* rSoles_;
        Vector3d* lSoles_;
        bool leftSwings_;
        bool rightSwings_;

        _Link* links_[13];

        Vector3d CoMEstimatorFK(double config[]);
        void updateState(double config[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer);
        void updateSolePosition();
        Vector3d getZMPLocal(Vector3d torque, double fz);
        Vector3d ZMPGlobal(Vector3d zmp_r, Vector3d zmp_l, double f_r, double f_l);

        ros::ServiceServer jntAngsServer_;
        ros::ServiceServer trajGenServer_;
        bool isTrajAvailable_;

        int index_;
        int size_;
};
