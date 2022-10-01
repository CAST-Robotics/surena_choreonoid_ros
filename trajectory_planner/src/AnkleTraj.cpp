#include "AnkleTraj.h"


AnkleTraj::AnkleTraj(bool use_file, double dt) : MinJerk(use_file, dt){
    if(!useFile_)
        cout << "Set Walking Parameters ..." << endl;
    else
        cout << "Set Config File Path ..." << endl;
}

void AnkleTraj::planStance(Vector3d r_foot, Vector3d l_foot, double time){
    int n = time / dt_;
    for(int i=0; i<n; i++){
        plannedLAnkle_.push_back(l_foot);
        plannedRAnkle_.push_back(r_foot);
    }
}

void AnkleTraj::planSteps(){
    vector<Vector3d> first_ankle;
    vector<Vector3d> second_ankle;

    for(int i=0; i<footStepCount_ - 2; i++){
        MatrixXd way_points{
            {footSteps_[i](0), footSteps_[i](0), (footSteps_[i+2](0) + footSteps_[i](0)) / 2, footSteps_[i+2](0), footSteps_[i+2](0)},
            {footSteps_[i](1), footSteps_[i](1), (footSteps_[i+2](1) + footSteps_[i](1)) / 2, footSteps_[i+2](1), footSteps_[i+2](1)},
            {footSteps_[i](2), footSteps_[i](2) + 0.02, footSteps_[i](2) + stepHeight_, footSteps_[i+2](2) + 0.02, footSteps_[i+2](2)}
            };
        MatrixXd vel_points = MatrixXd::Zero(3, 5);
        vel_points(0,2) = (footSteps_[i+2](0) - footSteps_[i](0)) / SSPDuration_;
        VectorXd time_points(5);
        time_points << 0.0, 0.2 * SSPDuration_, 0.5 * SSPDuration_, 0.8 * SSPDuration_, SSPDuration_;
        MatrixXd traj;
        this->cubicPolyTraj(way_points, time_points, 0.005, vel_points, traj);

        // first ankle swing
        if(i%2==0){
            // single support phase
            for(int j=0; j<int(SSPDuration_ / dt_); j++){
                first_ankle.push_back(traj.col(j));
                second_ankle.push_back(footSteps_[i+1]);
            }
            // double support phase
            for(int j=0; j<int(DSPDuration_ / dt_); j++){
                first_ankle.push_back(footSteps_[i+2]);
                second_ankle.push_back(footSteps_[i+1]);
            }
        }
        // second ankle swing
        else{
            // single support phase
            for(int j=0; j<int(SSPDuration_ / dt_); j++){
                second_ankle.push_back(traj.col(j));
                first_ankle.push_back(footSteps_[i+1]);
            }
            // double support phase
            for(int j=0; j<int(DSPDuration_ / dt_); j++){
                second_ankle.push_back(footSteps_[i+2]);
                first_ankle.push_back(footSteps_[i+1]);
            }
        }
    }

    if(leftFirst_){
        plannedLAnkle_ = first_ankle;
        plannedRAnkle_ = second_ankle;
    }else{
        plannedLAnkle_ = second_ankle;
        plannedRAnkle_ = first_ankle;
    }

    for(int i=0; i<plannedLAnkle_.size(); i++){
        cout << plannedLAnkle_[i](0) << ", " << plannedLAnkle_[i](1) << ", " << plannedLAnkle_[i](2) << ", ";
        cout << plannedRAnkle_[i](0) << ", " << plannedRAnkle_[i](1) << ", " << plannedRAnkle_[i](2) << endl;
    }
}