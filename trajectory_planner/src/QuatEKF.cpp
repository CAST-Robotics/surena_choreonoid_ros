#include "QuatEKF.h"

QuatEKF::QuatEKF() {

    Gravity_ = Vector3d(0, 0, -9.81);
    Matrix3d rotation_matrix = Matrix3d::Identity();
    GBaseRot_ = rotation_matrix;
    GBaseQuat_ = Quaterniond(rotation_matrix);

    GBasePos_ = Vector3d::Zero();
    GBaseVel_ = Vector3d::Zero();
    GRightFootPos_ = Vector3d::Zero();
    GLeftFootPos_ = Vector3d::Zero();
    BRightFootPos_ = Vector3d::Zero();
    BLeftFootPos_ = Vector3d::Zero();
    BAccBias_ = Vector3d::Zero();
    BGyroBias_ = Vector3d::Zero();

    BAcc_ = Vector3d::Zero();
    BGyro_ = Vector3d::Zero();

    BRFootMeasured_ = Vector3d::Zero();
    BLFootMeasured_ = Vector3d::Zero();

    statesDim_ = GBasePos_.size() + GBaseVel_.size() + 4 + GRightFootPos_.size() + 
                 GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    statesErrDim_ = GBasePos_.size() + GBaseVel_.size() + 3 + GRightFootPos_.size() + 
                    GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    rvqStatesDim_ = GBasePos_.size() + GBaseVel_.size() + 4;

    processNoiseDim_ = BAcc_.size() + BGyro_.size() + GRightFootPos_.size() + 
                       GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    measurmentDim_ = GRightFootPos_.size() + GLeftFootPos_.size();

    xPrior_ = MatrixXd::Zero(statesDim_, 1);
    xPosterior_ = MatrixXd::Zero(statesDim_, 1);
    y_ = MatrixXd::Zero(measurmentDim_, 1);
    z_ = MatrixXd::Zero(measurmentDim_, 1);
    deltaX_ = MatrixXd::Zero(statesDim_, 1);

    initStateVar_ = 50;
    P_ = initStateVar_ * MatrixXd::Identity(statesErrDim_, statesErrDim_);
    
    Lc_ = MatrixXd::Zero(statesErrDim_, processNoiseDim_);
    Qc_ = MatrixXd::Zero(processNoiseDim_, processNoiseDim_);
    Fc_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    Fk_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    Qk_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);

    Rk_ = MatrixXd::Zero(measurmentDim_, measurmentDim_);
    Hk_ = MatrixXd::Zero(measurmentDim_, statesErrDim_);
    Sk_ = MatrixXd::Zero(measurmentDim_, measurmentDim_);
    Kk_ = MatrixXd::Zero(statesErrDim_, measurmentDim_);

    // noise parameters initilization
    wf_ = 0.00078;
    ww_ = 0.000523;
    wp_r_ = 0.001;
    wp_l_ = 0.001;
    wbf_ = 0.0001;
    wbw_ = 0.000618;
    np_r_ = 0.01;
    np_l_ = 0.01;
    updateQc();
    
    dt_ = 0.002;
}

QuatEKF::~QuatEKF() {

}

void QuatEKF::setSensorData(Vector3d acc, Vector3d gyro){
    BAcc_ = acc;
    BGyro_ = gyro;
}

void QuatEKF::setMeasuredData(Vector3d r_kynematic, Vector3d l_kynematic){
    BRFootMeasured_ = r_kynematic;
    BLFootMeasured_ = l_kynematic;
    z_.segment(0, BRFootMeasured_.size()) = r_kynematic;
    z_.segment(BRFootMeasured_.size(), BLFootMeasured_.size()) = l_kynematic;
}

void QuatEKF::setState2prior(){
    GBasePos_ = xPrior_.segment(0, GBasePos_.size());
    GBaseVel_ = xPrior_.segment(GBasePos_.size(), GBaseVel_.size());
    GBaseQuat_.x() = xPrior_(GBasePos_.size() + GBaseVel_.size());
    GBaseQuat_.y() = xPrior_(GBasePos_.size() + GBaseVel_.size()  + 1);
    GBaseQuat_.z() = xPrior_(GBasePos_.size() + GBaseVel_.size()  + 2);
    GBaseQuat_.w() = xPrior_(GBasePos_.size() + GBaseVel_.size()  + 3);

    GBaseRot_ = GBaseQuat_.toRotationMatrix();

    GRightFootPos_ = xPrior_.segment(rvqStatesDim_, GRightFootPos_.size());
    GLeftFootPos_ = xPrior_.segment(rvqStatesDim_ + GRightFootPos_.size(), GLeftFootPos_.size());
    BAccBias_ = xPrior_.segment(rvqStatesDim_ + GRightFootPos_.size() + GLeftFootPos_.size(), BAccBias_.size());
    BGyroBias_ = xPrior_.segment(rvqStatesDim_ + GRightFootPos_.size() + GLeftFootPos_.size() + BAccBias_.size(), BGyroBias_.size());
}

void QuatEKF::setState2posterior(){
    GBasePos_ = xPosterior_.segment(0, GBasePos_.size());
    GBaseVel_ = xPosterior_.segment(GBasePos_.size(), GBaseVel_.size());
    GBaseQuat_.x() = xPosterior_(GBasePos_.size() + GBaseVel_.size());
    GBaseQuat_.y() = xPosterior_(GBasePos_.size() + GBaseVel_.size()  + 1);
    GBaseQuat_.z() = xPosterior_(GBasePos_.size() + GBaseVel_.size()  + 2);
    GBaseQuat_.w() = xPosterior_(GBasePos_.size() + GBaseVel_.size()  + 3);

    GBaseRot_ = GBaseQuat_.toRotationMatrix();

    GRightFootPos_ = xPosterior_.segment(rvqStatesDim_, GRightFootPos_.size());
    GLeftFootPos_ = xPosterior_.segment(rvqStatesDim_ + GRightFootPos_.size(), GLeftFootPos_.size());
    BAccBias_ = xPosterior_.segment(rvqStatesDim_ + GRightFootPos_.size() + GLeftFootPos_.size(), BAccBias_.size());
    BGyroBias_ = xPosterior_.segment(rvqStatesDim_ + GRightFootPos_.size() + GLeftFootPos_.size() + BAccBias_.size(), BGyroBias_.size());
}

void QuatEKF::predictState(){

    // remove bias from IMU data
    Vector3d BAcc_bar = BAcc_ - BAccBias_;
    Vector3d BGyro_bar = BGyro_ - BGyroBias_;
    // find orientation change between time steps (initialize quaternion with rotation vector)
    AngleAxisd delta_gyro((BGyro_bar * dt_).norm(), (BGyro_bar * dt_).normalized());
    Quaterniond delta_omega = Quaterniond(delta_gyro);

    // predict position and velocity
    xPrior_.segment(0,GBasePos_.size()) = GBasePos_ + dt_ * GBaseVel_ + (pow(dt_, 2) / 2) * (GBaseRot_ * BAcc_bar - Gravity_);
    xPrior_.segment(GBasePos_.size(), GBaseVel_.size()) = GBaseVel_ + dt_ * (GBaseRot_ * BAcc_bar - Gravity_);
    // predict orientation
    Quaterniond predicted_q = delta_omega * GBaseQuat_;

    xPrior_(GBasePos_.size() + GBaseVel_.size()) = predicted_q.x();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 1) = predicted_q.y();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 2) = predicted_q.z();
    xPrior_(GBasePos_.size() + GBaseVel_.size()  + 3) = predicted_q.w();

    // predict foot positions & biases
    xPrior_.segment(rvqStatesDim_, statesDim_ - rvqStatesDim_) = xPosterior_.segment(rvqStatesDim_, statesDim_ - rvqStatesDim_);
    //this->setState2prior();
}

void QuatEKF::updateLc(Matrix3d rot) {
    Lc_.block(3, 0, 3, 3) = -rot.transpose();
    Lc_.block(6, 3, 3, 3) = -Matrix3d::Identity();
    Lc_.block(9, 6, 3, 3) = rot.transpose(); // right foot
    Lc_.block(12, 9, 3, 3) = rot.transpose(); // left foot
    Lc_.block(15, 12, 3, 3) = Matrix3d::Identity();
    Lc_.block(18, 15, 3, 3) = Matrix3d::Identity();
}

void QuatEKF::updateQc() {
    Qc_.block(0, 0, 3, 3) = wf_ * Matrix3d::Identity();
    Qc_.block(3, 3, 3, 3) = ww_ * Matrix3d::Identity();
    Qc_.block(6, 6, 3, 3) = wp_r_ * Matrix3d::Identity();
    Qc_.block(9, 9, 3, 3) = wp_l_ * Matrix3d::Identity();
    Qc_.block(12, 12, 3, 3) = wbf_ * Matrix3d::Identity();
    Qc_.block(15, 15, 3, 3) = wbw_ * Matrix3d::Identity();
}

Matrix3d QuatEKF::skewSym(const Vector3d &vec){
    Matrix3d skew;
    skew << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return skew;
}

void QuatEKF::updateFc(Matrix3d rot) {
    Fc_.block(0, 3, 3, 3) = Matrix3d::Identity();
    Fc_.block(3, 6, 3, 3) = -rot.transpose() * skewSym(BAcc_);
    Fc_.block(3, 15, 3, 3) = -rot.transpose();
    Fc_.block(6, 6, 3, 3) = -skewSym(BGyro_);
    Fc_.block(6, 18, 3, 3) = -Matrix3d::Identity();
}

void QuatEKF::predictCov() {
    this->updateFc(GBaseRot_);
    this->updateLc(GBaseRot_);
    this->updateQc();

    Fk_ = MatrixXd::Identity(statesErrDim_, statesErrDim_) + Fc_ * dt_;
    Qk_ = Fk_ * Lc_ * Qc_ * Lc_.transpose() * Fk_.transpose() * dt_;
    P_ = Fk_ * P_ * Fk_.transpose() + Qk_;
}

void QuatEKF::predict() {
    this->predictState();
    this->predictCov();
    this->setState2prior();
}

void QuatEKF::updateRk() {
    Rk_.block(0, 0, 3, 3) = np_r_ * Matrix3d::Identity();
    Rk_.block(3, 3, 3, 3) = np_l_ * Matrix3d::Identity();
    Rk_ = Rk_  / dt_;
}

void QuatEKF::updateHk() {
    Hk_.block(0, 0, 3, 3) = -GBaseRot_;
    BRightFootPos_ = GBaseRot_ * (GRightFootPos_ - GBasePos_);
    Hk_.block(0, 6, 3, 3) = this->skewSym(BRightFootPos_);
    Hk_.block(0, 9, 3, 3) = GBaseRot_;
    Hk_.block(3, 0, 3, 3) = -GBaseRot_;
    BLeftFootPos_ = GBaseRot_ * (GLeftFootPos_ - GBasePos_);
    Hk_.block(3, 6, 3, 3) = this->skewSym(BLeftFootPos_);
    Hk_.block(3, 12, 3, 3) = GBaseRot_;
}


void QuatEKF::update() {
    this->updateRk();
    this->updateHk();
    Sk_ = Hk_ * P_ * Hk_.transpose() + Rk_;

    // calculate Kalman gain
    Kk_ = P_ * Hk_.transpose() * (Sk_).inverse();

    // calculate Residual
    y_.head(GRightFootPos_.size()) = BRFootMeasured_ - BRightFootPos_;
    y_.segment(GRightFootPos_.size(), GLeftFootPos_.size()) = BLFootMeasured_ - BLeftFootPos_;

    // update state
    deltaX_ = Kk_ * y_;
    xPosterior_.head(GBasePos_.size() + GBaseVel_.size()) += deltaX_.head(GBasePos_.size() + GBaseVel_.size());
    xPosterior_.tail(GRightFootPos_.size() + GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size()) += deltaX_.tail(GRightFootPos_.size() + GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size());

    Vector3d delta_phi;
    delta_phi(0) = xPosterior_(GBasePos_.size() + GBaseVel_.size());
    delta_phi(1) = xPosterior_(GBasePos_.size() + GBaseVel_.size() + 1);
    delta_phi(2) = xPosterior_(GBasePos_.size() + GBaseVel_.size() + 2);
    AngleAxisd delta_rot(delta_phi.norm(), delta_phi.normalized());
    Quaterniond delta_quat = Quaterniond(delta_rot);
    Quaterniond q_posterior = delta_quat * GBaseQuat_;
    xPosterior_(GBasePos_.size() + GBaseVel_.size()) = q_posterior.x();
    xPosterior_(GBasePos_.size() + GBaseVel_.size() + 1) = q_posterior.y();
    xPosterior_(GBasePos_.size() + GBaseVel_.size() + 2) = q_posterior.z();
    xPosterior_(GBasePos_.size() + GBaseVel_.size() + 3) = q_posterior.w();

    // update covariance
    P_ = (MatrixXd::Identity(statesErrDim_, statesErrDim_) - Kk_ * Hk_) * P_;

    this->setState2posterior();
}

void QuatEKF::runFilter(Vector3d acc, Vector3d gyro, Vector3d rfmeasured, Vector3d lfmeasured) {
    this->setSensorData(acc, gyro);
    this->predict();
    cout << GBasePos_(0) << ", " << GBasePos_(1) << ", " << GBasePos_(2) << ", ";
    cout << GBaseVel_(0) << ", " << GBaseVel_(1) << ", " << GBaseVel_(2) << ", ";
    cout << GBaseQuat_.x() << ", " << GBaseQuat_.y() << ", " << GBaseQuat_.z() << ", " << GBaseQuat_.w() << endl;
    setMeasuredData(rfmeasured, lfmeasured);
    this->update();
}

// int main() {
//     EKFEstimator estimator;
//     Vector3d acc(0, 0, 9.9);
//     Vector3d gyro(0, 0, 0);
//     estimator.runFilter(acc, gyro, gyro, gyro);
//     return 0;
// }