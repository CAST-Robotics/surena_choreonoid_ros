#include "EKFEstimator.h"

EKFEstimator::EKFEstimator() {
    Matrix3d rotation_matrix = Matrix3d::Identity();
    GBaseAtitude_ = Quaterniond(rotation_matrix);
    statesDim_ = GBasePos_.size() + GBaseVel_.size() + 4 + GRightFootPos_.size() + 
                    GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();
     
    rvqStatesDim_ = GBasePos_.size() + GBaseVel_.size() + 4;
    
    xPrior_ = MatrixXd::Zero(statesDim_, 1);
    xPosterior_ = MatrixXd::Zero(statesDim_, 1);

    dt_ = 0.005;
}

EKFEstimator::~EKFEstimator() {

}

void EKFEstimator::setSensorData(Vector3d acc, Vector3d gyro){

}

void EKFEstimator::predict() {
    predictState();
}

void EKFEstimator::predictState(){
    // remove bias from IMU data
    Vector3d BAcc_bar = BAcc_ - BAccBias_;
    Vector3d BGyro_bar = BGyro_ - BGyroBias_;
    // find orientation change between time steps (initialize quaternion with rotation vector)
    Quaterniond delta_omega = Quaterniond(BGyro_bar * dt_);

    // predict position and velocity
    xPrior_.segment(0,GBasePos_.size()) = GBasePos_ + dt_ * GBaseVel_ + (pow(dt_, 2) / 2) * (GBaseRot_ * BAcc_bar + Gravity_);
    xPrior_.segment(GBasePos_.size(), GBaseVel_.size()) = GBaseVel_ + dt_ * (GBaseRot_ * BAcc_bar + Gravity_);
    // predict orientation
    Quaterniond predicted_q = delta_omega * GBaseAtitude_;

    xPrior_(GBasePos_.size() + GBaseVel_.size()) = predicted_q.x();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 1) = predicted_q.y();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 2) = predicted_q.z();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 3) = predicted_q.w();

    // predict foot positions & biases
    xPrior_(rvqStatesDim_, statesDim_ - rvqStatesDim_) = xPosterior_(rvqStatesDim_, statesDim_ - rvqStatesDim_);
}


void EKFEstimator::update() {

}

void EKFEstimator::updateState() {
    
}

