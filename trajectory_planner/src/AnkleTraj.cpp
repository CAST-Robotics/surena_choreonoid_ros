#include "AnkleTraj.h"


AnkleTraj::AnkleTraj(bool use_file, double dt) : MinJerk(use_file, dt){
    if(!useFile_)
        cout << "Set Walking Parameters ..." << endl;
    else
        cout << "Set Config File Path ..." << endl;
}

void AnkleTraj::planStance(Vector3d r_foot, Vector3d l_foot, double yaw, double time){
    int n = time / dt_;
    // we assume that left and right yaw angles are identical in the stance phase
    //TODO: different yaw in the stance
    Matrix3d rot = Matrix3d(AngleAxisd(yaw, Vector3d::UnitZ()));
    for(int i=0; i<n; i++){
        plannedLAnklePos_.push_back(l_foot);
        plannedRAnklePos_.push_back(r_foot);
        plannedLAnkleRot_.push_back(rot);
        plannedRAnkleRot_.push_back(rot);
    }
}

void AnkleTraj::planInitialDSP(){
    if(leftFirst_)
        this->planStance(footSteps_[1], footSteps_[0], footYaws_[0], initDSPDuration_);
    else
        this->planStance(footSteps_[0], footSteps_[1], footYaws_[0], initDSPDuration_);
}

void AnkleTraj::planFinalDSP(){
    if(footSteps_[footSteps_.size()-1](1)>0)
        this->planStance(footSteps_[footSteps_.size()-2], footSteps_[footSteps_.size()-1], footYaws_[footYaws_.size()-1], finalDSPDuration_);
    else
        this->planStance(footSteps_[footSteps_.size()-1], footSteps_[footSteps_.size()-2], footYaws_[footYaws_.size()-1], finalDSPDuration_);
}

void AnkleTraj::planSteps(){
    vector<Vector3d> first_ankle_pos;
    vector<Matrix3d> first_ankle_rot;
    vector<Vector3d> second_ankle_pos;
    vector<Matrix3d> second_ankle_rot;

    for(int i=0; i<footStepCount_ - 2; i++){
        MatrixXd way_points{
            {footSteps_[i](0), footSteps_[i](0), (footSteps_[i+2](0) + footSteps_[i](0)) / 2, footSteps_[i+2](0), footSteps_[i+2](0)},
            {footSteps_[i](1), footSteps_[i](1), (footSteps_[i+2](1) + footSteps_[i](1)) / 2, footSteps_[i+2](1), footSteps_[i+2](1)},
            {footSteps_[i](2), footSteps_[i](2) + 0.02, footSteps_[i](2) + stepHeight_, footSteps_[i+2](2) + 0.02, footSteps_[i+2](2)}
            };
        MatrixXd vel_points = MatrixXd::Zero(3, 5);
        vel_points(0, 2) = (footSteps_[i+2](0) - footSteps_[i](0)) / SSPDuration_;
        VectorXd time_points(5);
        time_points << 0.0, 0.2 * SSPDuration_, 0.5 * SSPDuration_, 0.8 * SSPDuration_, SSPDuration_;
        MatrixXd traj;
        this->cubicPolyTraj(way_points, time_points, dt_, vel_points, traj);

        MatrixXd yaw_way_points{{footYaws_[i], footYaws_[i+2]}};
        MatrixXd yaw_vel_points = MatrixXd::Zero(1, 2);
        VectorXd yaw_time_points(2);
        yaw_time_points << 0.0, SSPDuration_;
        MatrixXd yaw_traj;
        this->cubicPolyTraj(yaw_way_points, yaw_time_points, dt_, yaw_vel_points, yaw_traj);

        // first ankle swing
        if(i%2==0){
            // single support phase
            for(int j=0; j<int(SSPDuration_ / dt_); j++){
                first_ankle_pos.push_back(Vector3d(traj(0, j), traj(1, j), traj(2, j)));
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(yaw_traj(0, j), Vector3d::UnitZ())));
                second_ankle_pos.push_back(footSteps_[i+1]);
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
            // double support phase
            for(int j=0; j<int(DSPDuration_ / dt_); j++){
                first_ankle_pos.push_back(footSteps_[i+2]);
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+2], Vector3d::UnitZ())));
                second_ankle_pos.push_back(footSteps_[i+1]);
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
        }
        // second ankle swing
        else{
            // single support phase
            for(int j=0; j<int(SSPDuration_ / dt_); j++){
                second_ankle_pos.push_back(Vector3d(traj(0, j), traj(1, j), traj(2, j)));
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(yaw_traj(0, j), Vector3d::UnitZ())));
                first_ankle_pos.push_back(footSteps_[i+1]);
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
            // double support phase
            for(int j=0; j<int(DSPDuration_ / dt_); j++){
                second_ankle_pos.push_back(footSteps_[i+2]);
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+2], Vector3d::UnitZ())));
                first_ankle_pos.push_back(footSteps_[i+1]);
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
        }
    }

    if(leftFirst_){
        plannedLAnklePos_.insert(plannedLAnklePos_.end(), first_ankle_pos.begin(), first_ankle_pos.end());
        plannedLAnkleRot_.insert(plannedLAnkleRot_.end(), first_ankle_rot.begin(), first_ankle_rot.end());
        plannedRAnklePos_.insert(plannedRAnklePos_.end(), second_ankle_pos.begin(), second_ankle_pos.end());
        plannedRAnkleRot_.insert(plannedRAnkleRot_.end(), second_ankle_rot.begin(), second_ankle_rot.end());
    }else{
        plannedLAnklePos_.insert(plannedRAnklePos_.end(), second_ankle_pos.begin(), second_ankle_pos.end());
        plannedLAnkleRot_.insert(plannedLAnkleRot_.end(), second_ankle_rot.begin(), second_ankle_rot.end());
        plannedRAnklePos_.insert(plannedLAnklePos_.end(), first_ankle_pos.begin(), first_ankle_pos.end());
        plannedRAnkleRot_.insert(plannedRAnkleRot_.end(), first_ankle_rot.begin(), first_ankle_rot.end());
    }

    // for(int i=0; i<plannedLAnklePos_.size(); i++){
    //     cout << plannedLAnklePos_[i](0) << ", " << plannedLAnklePos_[i](1) << ", " << plannedLAnklePos_[i](2) << ", ";
    //     cout << plannedRAnklePos_[i](0) << ", " << plannedRAnklePos_[i](1) << ", " << plannedRAnklePos_[i](2) << endl;
    // }
}