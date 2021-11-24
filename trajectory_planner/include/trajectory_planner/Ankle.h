# pragma once

#include "MinJerk.h"

class Ankle: private MinJerk{
    public:
        Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt, double theta);
        ~Ankle();
        void updateFoot(Vector3d foot_pose[]);
        void generateTrajectory(); 
        Vector3d* getTrajectoryL();
        Vector3d* getTrajectoryR();
        Matrix3d* getRotTrajectoryR();
        Matrix3d* getRotTrajectoryL();

    private:
        double tStep_;
        double tDS_;
        double dt_;
        short int num_step;  //؟؟؟
        double alpha_;
        int stepCount_;
        bool leftFirst_;
        double height_;
        double theta_;

        Vector3d* footPose_;
        Vector3d* lFoot_;
        Vector3d* rFoot_;
        Matrix3d* lFootRot_;
        Matrix3d* rFootRot_;
       
        void updateTrajectory(bool left_first);
        Matrix3d yawRotMat(double theta);
};