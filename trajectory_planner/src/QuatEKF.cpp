#include "QuatEKF.h"

QuatEKF::QuatEKF() {

    x_ = MatrixXd::Zero(statesDim_, 1);
    xPrev_ = MatrixXd::Zero(statesDim_, 1);

    // default initialize states
    this->initializeStates(Quaterniond(Matrix3d::Identity()), Vector3d::Zero(), Vector3d::Zero(),
                                       Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());

    Gravity_ = Vector3d(0, 0, -9.81);
    
    BAcc_ = Vector3d::Zero();
    BGyro_ = Vector3d::Zero();

    BRFootMeasured_ = Vector3d::Zero();
    BLFootMeasured_ = Vector3d::Zero();

    statesDim_ = 4 + GBaseVel_.size() + GBasePos_.size() + GLeftFootPos_.size() + 
                 GRightFootPos_.size() + BGyroBias_.size() + BAccBias_.size();

    statesErrDim_ = 3 + GBaseVel_.size() + GBasePos_.size() + GLeftFootPos_.size() + 
                    GRightFootPos_.size() + BGyroBias_.size() + BAccBias_.size();

    qvrStatesDim_ = 4 + GBaseVel_.size() + GBasePos_.size();

    // processNoiseDim_ = BAcc_.size() + BGyro_.size() + GRightFootPos_.size() + 
    //                    GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    measurmentDim_ = GLeftFootPos_.size() + GRightFootPos_.size();

    y_ = MatrixXd::Zero(measurmentDim_, 1);
    z_ = MatrixXd::Zero(measurmentDim_, 1);
    deltaX_ = MatrixXd::Zero(statesDim_, 1);

    P_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    this->initializeCovariance(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
    
    this->setDt(0.002);

    this->setNoiseStd(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

    
}

QuatEKF::~QuatEKF() {

}

void QuatEKF::initializeStates(Quaterniond q, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                               Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias){
    GBaseQuat_ = q;
    GBaseRot_ = q.toRotationMatrix();
    GBaseVel_ = base_vel;
    GBasePos_ = base_pos;
    GLeftFootPos_ = lf_pos;
    GRightFootPos_ = rf_pos;
    BGyroBias_ = gyro_bias;
    BAccBias_ = acc_bias;
    concatStates(q, base_vel, base_pos, lf_pos, rf_pos, gyro_bias, acc_bias, xPrev_);
}

void QuatEKF::concatStates(Quaterniond q, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                          Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias, VectorXd &output){
    output(0) = q.w();
    output(1) = q.x();
    output(2) = q.y();
    output(3) = q.z();

    output.segment(4, GBaseVel_.size()) = base_vel;
    output.segment(4 + GBaseVel_.size(), GBasePos_.size()) = base_pos;
    output.segment(qvrStatesDim_, GLeftFootPos_.size()) = lf_pos;
    output.segment(qvrStatesDim_ + GLeftFootPos_.size(), GRightFootPos_.size()) = rf_pos;
    output.segment(qvrStatesDim_ + GLeftFootPos_.size() + GRightFootPos_.size(), BGyroBias_.size()) = gyro_bias;
    output.segment(qvrStatesDim_ + GLeftFootPos_.size() + GRightFootPos_.size() + BGyroBias_.size(), BAccBias_.size()) = acc_bias;
}

void QuatEKF::initializeCovariance(double quat_std, double vel_std, double pos_std,
                                   double contact_std, double gyro_std, double acc_std){
    quatStd_ = quat_std;
    velStd_ = vel_std;
    posStd_ = pos_std;
    contactStd_ = contact_std;
    gyroStd_ = gyro_std;
    accStd_ = acc_std;

    P_.block(0, 0, 3, 3) = quatStd_ * MatrixXd::Identity(3, 3);
    P_.block(3, 3, 3, 3) = velStd_ * MatrixXd::Identity(3, 3);
    P_.block(6, 6, 3, 3) = posStd_ * MatrixXd::Identity(3, 3);
    P_.block(9, 9, 3, 3) = contactStd_ * MatrixXd::Identity(3, 3);
    P_.block(12, 12, 3, 3) = contactStd_ * MatrixXd::Identity(3, 3);
    P_.block(15, 15, 3, 3) = gyroStd_ * MatrixXd::Identity(3, 3);
    P_.block(18, 18, 3, 3) = accStd_ * MatrixXd::Identity(3, 3);
}

void QuatEKF::setNoiseStd(double gyro_noise, double acc_noise, double gyro_bias_noise, 
                         double acc_bias_noise, double contact_noise, double measurement_noise){
    gyroNoiseStd_ = gyro_noise;
    accNoiseStd_ = acc_noise;
    gyroBiasNoiseStd_ = gyro_bias_noise;
    accBiasNoiseStd_ = acc_bias_noise;
    contactNoiseStd_ = contact_noise;
    measurementNoiseStd_ = measurement_noise;
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

void QuatEKF::seprateStates(VectorXd &x){

    GBaseQuat_.x() = x(0);
    GBaseQuat_.y() = x(1);
    GBaseQuat_.z() = x(2);
    GBaseQuat_.w() = x(3);
    GBaseVel_ = x.segment(4, GBaseVel_.size());
    GBasePos_ = x.segment(4 + GBaseVel_.size(), GBasePos_.size());
    GLeftFootPos_ = x.segment(qvrStatesDim_, GLeftFootPos_.size());
    GRightFootPos_ = x.segment(qvrStatesDim_ + GLeftFootPos_.size(), GRightFootPos_.size());
    BGyroBias_ = x.segment(qvrStatesDim_ + GLeftFootPos_.size() + GRightFootPos_.size(), BGyroBias_.size());
    BAccBias_ = x.segment(qvrStatesDim_ + GLeftFootPos_.size() + GRightFootPos_.size() + BGyroBias_.size(), BAccBias_.size());

    GBaseRot_ = GBaseQuat_.toRotationMatrix();
}

void QuatEKF::predictState(){
    
    this->seprateStates(xPrev_);
    // remove bias from IMU data
    BAcc_ = BAcc_ - BAccBias_;
    BGyro_ = BGyro_ - BGyroBias_;

    Matrix3d acc_skew = skewSym(BAcc_);
    
    Matrix3d gamma_0;
    gamma(dt_ * BGyro_, 0, gamma_0);
    Matrix3d gamma_1;
    gamma(dt_ * BGyro_, 1, gamma_1);
    Matrix3d gamma_2;
    gamma(dt_ * BGyro_, 2, gamma_2);

    // find orientation change between time steps (initialize quaternion with rotation vector)
    AngleAxisd delta_gyro((dt_ * BGyro_).norm(), (dt_ * BGyro_).normalized());
    Quaterniond delta_omega = Quaterniond(delta_gyro);
    // predict orientation
    Quaterniond predicted_q = delta_omega * GBaseQuat_;
    Matrix3d predicted_R = predicted_q.toRotationMatrix();

    // predict position and velocity
    Vector3d predicted_v = GBaseVel_ + dt_ * (GBaseRot_ * gamma_1 * BAcc_ + Gravity_);
    Vector3d predicted_p = GBasePos_ + dt_ * GBaseVel_ + pow(dt_, 2) * (GBaseRot_ * gamma_2 * BAcc_ + 0.5 * Gravity_);

    // predict foot positions
    // what happens if the measurement is not available?
    Vector3d predicted_lf_p;
    Vector3d predicted_rf_p;
    if(updateEnabled_){
        predicted_lf_p = predicted_p + predicted_R * BLFootMeasured_;
        predicted_rf_p = predicted_p + predicted_R * BRFootMeasured_;
    }
    predicted_lf_p = contact_[0] * GLeftFootPos_ + (1 - contact_[0]) * predicted_lf_p;
    predicted_rf_p = contact_[1] * GRightFootPos_ + (1 - contact_[1]) * predicted_rf_p;

    // predict biases
    Vector3d predicted_gyro_bias = BGyroBias_;
    Vector3d predicted_acc_bias = BAccBias_;

    this->concatStates(predicted_q, predicted_v, predicted_p, predicted_lf_p, predicted_rf_p, predicted_gyro_bias, predicted_acc_bias, x_);
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