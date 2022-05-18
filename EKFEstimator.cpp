#include "EKFEstimator.h"

explicit EKFEstimator::EKFEstimator() {
    Matrix3d rotation_matrix = Matrix3d::Identity();
    GBaseRot_ = rotation_matrix;
    GBaseQuat_ = Quaterniond(rotation_matrix);

    statesDim_ = GBasePos_.size() + GBaseVel_.size() + 4 + GRightFootPos_.size() + 
                 GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    statesErrDim_ = GBasePos_.size() + GBaseVel_.size() + 3 + GRightFootPos_.size() + 
                    GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    rvqStatesDim_ = GBasePos_.size() + GBaseVel_.size() + 4;

    processNoiseDim_ = BAcc_.size() + BGyro_.size() + GRightFootPos_.size() + 
                       GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    xPrior_ = MatrixXd::Zero(statesDim_, 1);
    xPosterior_ = MatrixXd::Zero(statesDim_, 1);
    Lc_ = MatrixXd::Zero(statesErrDim_, processNoiseDim_);
    Qc_ = MatrixXd::Zero(processNoiseDim_, processNoiseDim_);
    Fc_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    Fk_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    Qk_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);

    // noise parameters initilization
    wf_ = 0.00078;
    ww_ = 0.000523;
    wp_r_ = 0.001;
    wp_l_ = 0.001;
    wbf_ = 0.0001;
    wbw_ = 0.000618;
    updateQc();
    
    dt_ = 0.005;
}

EKFEstimator::~EKFEstimator() {

}

void EKFEstimator::setSensorData(Vector3d acc, Vector3d gyro){

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
    Quaterniond predicted_q = delta_omega * GBaseQuat_;

    xPrior_(GBasePos_.size() + GBaseVel_.size()) = predicted_q.x();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 1) = predicted_q.y();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 2) = predicted_q.z();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 3) = predicted_q.w();

    // predict foot positions & biases
    xPrior_(rvqStatesDim_, statesDim_ - rvqStatesDim_) = xPosterior_(rvqStatesDim_, statesDim_ - rvqStatesDim_);
}

void EKFEstimator::updateLc(Matrix3d rot) {
    Lc_.block(3, 0, 3, 3) = -rot.transpose();
    Lc_.block(6, 3, 3, 3) = -Matrix3d::Identity();
    Lc_.block(9, 6, 3, 3) = rot.transpose(); // right foot
    Lc_.block(12, 9, 3, 3) = rot.transpose(); // left foot
    Lc_.block(15, 12, 3, 3) = Matrix3d::Identity();
    Lc_.block(18, 15, 3, 3) = Matrix3d::Identity();
}

void EKFEstimator::updateQc() {
    Qc_.block(0, 0, 3, 3) = wf_ * Matrix3d::Identity();
    Qc_.block(3, 3, 3, 3) = ww_ * Matrix3d::Identity();
    Qc_.block(6, 6, 3, 3) = wp_r_ * Matrix3d::Identity();
    Qc_.block(9, 9, 3, 3) = wp_l_ * Matrix3d::Identity();
    Qc_.block(12, 12, 3, 3) = wbf_ * Matrix3d::Identity();
    Qc_.block(15, 15, 3, 3) = wbw_ * Matrix3d::Identity();
}

Matrix3d EKFEstimator::skewSym(const Vector3d &vec){
    Matrix3d skew;
    skew << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return skew;
}

void EKFEstimator::updateFc(Matrix3d rot) {
    Fc_.block(0, 3, 3, 3) = Matrix3d::Identity();
    Fc_.block(3, 6, 3, 3) = rot.transpose() * skewSym(BAcc_);
    Fc_.block(3, 15, 3, 3) = -rot.transpose();
    Fc_.block(6, 6, 3, 3) = -skewSym(BGyro_);
    Fc_.block(6, 18, 3, 3) = -Matrix3d::Identity();
}

void EKFEstimator::predictCov() {
    this->updateFc(GBaseRot_);
    this->updateLc(GBaseRot_);
    this->updateQc();

    Fk_ = MatrixXd::Identity(statesErrDim_, statesErrDim_) + Fc_ * dt_;
    Qk_ = Fk_ * Lc_ * Qc_ * Lc_.transpose() * Fk_.transpose() * dt_;
    P_ = Fk_ * P_ * Fk_.transpose() + Qk_;
}

void EKFEstimator::predict() {
    this->predictState();
    this->predictCov();
}

void EKFEstimator::update() {

}

void EKFEstimator::updateState() {

}

