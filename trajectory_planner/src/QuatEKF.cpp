#include "QuatEKF.h"

QuatEKF::QuatEKF() {

    x_ = MatrixXd::Zero(statesDim_, 1);
    xPrev_ = MatrixXd::Zero(statesDim_, 1);

    // default initialize states
    this->initializeStates(Quaterniond(Matrix3d::Identity()), Vector3d::Zero(), Vector3d::Zero(),
                                       Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());

    Gravity_ = Vector3d(0, 0, -9.81);
    
    BGyro_ = Vector3d::Zero();
    BAcc_ = Vector3d::Zero();

    BLFootMeasured_ = Vector3d::Zero();
    BRFootMeasured_ = Vector3d::Zero();
    BLFootRotMeasured_ = Matrix3d::Identity();
    BRFootRotMeasured_ = Matrix3d::Identity();

    statesDim_ = 4 + GBaseVel_.size() + GBasePos_.size() + GLeftFootPos_.size() + 
                 GRightFootPos_.size() + BGyroBias_.size() + BAccBias_.size();

    statesErrDim_ = 3 + GBaseVel_.size() + GBasePos_.size() + GLeftFootPos_.size() + 
                    GRightFootPos_.size() + BGyroBias_.size() + BAccBias_.size();

    qvrStatesDim_ = 4 + GBaseVel_.size() + GBasePos_.size();

    // processNoiseDim_ = BAcc_.size() + BGyro_.size() + GRightFootPos_.size() + 
    //                    GLeftFootPos_.size() + BAccBias_.size() + BGyroBias_.size();

    measurmentDim_ = GLeftFootPos_.size() + GRightFootPos_.size();

    y_ = MatrixXd::Zero(measurmentDim_, 1);
    deltaX_ = MatrixXd::Zero(statesErrDim_, 1);

    Phi_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    Qd_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);

    P_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);

    Hd_ = MatrixXd::Zero(measurmentDim_, statesErrDim_);
    Rd_ = MatrixXd::Zero(measurmentDim_, measurmentDim_);
    Sd_ = MatrixXd::Zero(measurmentDim_, measurmentDim_);
    Kd_ = MatrixXd::Zero(statesErrDim_, measurmentDim_);
    
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
    
    Matrix3d gamma_0 = gamma(dt_ * BGyro_, 0);
    Matrix3d gamma_1 = gamma(dt_ * BGyro_, 1);
    Matrix3d gamma_2 = gamma(dt_ * BGyro_, 2);

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

void QuatEKF::predictCov() {

    Matrix3d gamma_0 = gamma(dt_ * BGyro_, 0);
    Matrix3d gamma_1 = gamma(dt_ * BGyro_, 1);
    Matrix3d gamma_2 = gamma(dt_ * BGyro_, 2);
    
    Matrix3d phi_skew = skewSym(dt_ * BGyro_);
    Matrix3d acc_skew = skewSym(BAcc_);
    double phi_norm = (dt_ * BGyro_).norm();
    Matrix3d psi_1;
    Matrix3d psi_2;

    if(phi_norm > 1e-6){

        psi_1 = acc_skew * gamma(-dt_ * BGyro_, 2)
        + ((sin(phi_norm) - phi_norm * cos(phi_norm))/(pow(phi_norm, 3))) * (phi_skew * acc_skew)
        - ((cos(2 * phi_norm) - 4 * cos(phi_norm) + 3) / (4 * pow(phi_norm, 4)))*(phi_skew * acc_skew * phi_skew)
        + ((4 * sin(phi_norm) + sin(2 * phi_norm) - 4 * phi_norm * cos(phi_norm) - 2 * phi_norm) / (4 * pow(phi_norm, 5))) * (phi_skew * acc_skew * phi_skew * phi_skew)
        + ((pow(phi_norm, 2) - 2 * phi_norm * sin(phi_norm) - 2 * cos(phi_norm) + 2) / (2 * pow(phi_norm, 4))) * (phi_skew * phi_skew * acc_skew)
        - ((6 * phi_norm - 8 * sin(phi_norm) + sin(2 * phi_norm)) / (4 * pow(phi_norm, 5))) * (phi_skew * phi_skew * acc_skew * phi_skew)
        + ((2 * pow(phi_norm, 2) - 4 * phi_norm * sin(phi_norm) - cos(2 * phi_norm) + 1) / (4 * pow(phi_norm, 6))) * (phi_skew * phi_skew * acc_skew * phi_skew * phi_skew);

        psi_2 = acc_skew * gamma(-dt_ * BGyro_, 3)
        - ((phi_norm * sin(phi_norm) + 2 * cos(phi_norm) - 2) / (pow(phi_norm, 4))) * (phi_skew * acc_skew)
        - ((6 * phi_norm - 8 * sin(phi_norm) + sin(2 * phi_norm)) / (8 * pow(phi_norm, 5))) * (phi_skew * acc_skew * phi_skew)
        - ((2 * pow(phi_norm, 2) + 8 * phi_norm * sin(phi_norm) + 16 * cos(phi_norm) + cos(2 * phi_norm) - 17) / (8 * pow(phi_norm, 6))) * (phi_skew * acc_skew * phi_skew * phi_skew)
        + ((pow(phi_norm, 3) + 6 * phi_norm - 12 * sin(phi_norm) + 6 * phi_norm * cos(phi_norm)) / (6 * pow(phi_norm, 5))) * (phi_skew * phi_skew * acc_skew)
        - ((6 * pow(phi_norm, 2) + 16 * cos(phi_norm) - cos(2 * phi_norm) - 15) / (8 * pow(phi_norm, 6))) * (phi_skew * phi_skew * acc_skew * phi_skew)
        + ((4 * pow(phi_norm, 3) + 6 * phi_norm - 24 * sin(phi_norm) - 3 * sin(2 * phi_norm) + 24 * phi_norm * cos(phi_norm)) / (24 * pow(phi_norm, 7))) * (phi_skew * phi_skew * acc_skew * phi_skew * phi_skew);
    
    }else{
        psi_1 = (1.0/2.0) * acc_skew;
        psi_2 = (1.0/6.0) * acc_skew;
    }

    Phi_.block(0, 0, 3, 3) = gamma_0.transpose();
    Phi_.block(3, 0, 3, 3) = -dt_ * GBaseRot_ * skewSym(gamma_1 * BAcc_);
    Phi_.block(6, 0, 3, 3) = -pow(dt_, 2) * GBaseRot_ * skewSym(gamma_2 * BAcc_);
    Phi_.block(6, 3, 3, 3) = dt_ * Matrix3d::Identity();
    Phi_.block(0, 15, 3, 3) = -dt_ * gamma_1.transpose();
    Phi_.block(3, 15, 3, 3) = -pow(dt_, 2) * GBaseRot_ * psi_1;
    Phi_.block(6, 15, 3, 3) = -pow(dt_, 3) * GBaseRot_ * psi_2;
    Phi_.block(3, 18, 3, 3) = -dt_ * GBaseRot_ * gamma_1;
    Phi_.block(6, 18, 3, 3) = -pow(dt_, 2) * GBaseRot_ * gamma_2;

    this->updateQd();

    P_ = Phi_ * P_ * Phi_.transpose() + Qd_;
}

void QuatEKF::updateQd() {
    Matrix3d hR_L = BLFootRotMeasured_;
    Matrix3d hR_R = BRFootRotMeasured_;
    Qd_.block(0, 0, 3, 3) = gyroNoiseStd_ * Matrix3d::Identity();
    Qd_.block(3, 3, 3, 3) = accNoiseStd_ * Matrix3d::Identity();
    Qd_.block(6, 6, 3, 3) = Matrix3d::Zero();
    Qd_.block(9, 9, 3, 3) = hR_L * (contactNoiseStd_ * Matrix3d::Identity() + (1e4 * (1 - contact_[0])) * Matrix3d::Identity()) * hR_L.transpose();
    Qd_.block(12, 12, 3, 3) = hR_R * (contactNoiseStd_ * Matrix3d::Identity() + (1e4 * (1 - contact_[1])) * Matrix3d::Identity()) * hR_R.transpose();
    Qd_.block(15, 15, 3, 3) = gyroBiasNoiseStd_ * Matrix3d::Identity();
    Qd_.block(18, 18, 3, 3) = accBiasNoiseStd_ * Matrix3d::Identity();

    MatrixXd G = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    G.block(0, 0, 3, 3) = Matrix3d::Identity();
    G.block(3, 3, 3, 3) = GBaseRot_;
    G.block(6, 6, 3, 3) = Matrix3d::Zero();
    G.block(9, 9, 3, 3) = GBaseRot_;
    G.block(12, 12, 3, 3) = GBaseRot_;
    G.block(15, 15, 3, 3) = Matrix3d::Identity();
    G.block(18, 18, 3, 3) = Matrix3d::Identity();

    Qd_ = Phi_ * G * Qd_ * G.transpose() * Phi_.transpose() * dt_;
}

void QuatEKF::predict() {
    this->predictState();
    this->predictCov();
}

void QuatEKF::updateState(VectorXd delta, VectorXd &x){
    this->seprateStates(x);
    AngleAxisd delta_rot((x.segment(0, 3)).norm(), (x.segment(0, 3)).normalized());
    Quaterniond delta_quat = Quaterniond(delta_rot);
    Quaterniond updated_q = delta_quat * GBaseQuat_;
    Vector3d updated_v = GBaseVel_ + delta.segment(3, 3);
    Vector3d updated_p = GBasePos_ + delta.segment(6, 3);
    Vector3d updated_lfp = GLeftFootPos_ + delta.segment(9, 3);
    Vector3d updated_rfp = GRightFootPos_ + delta.segment(12, 3);
    Vector3d updated_bg = BGyroBias_ + delta.segment(15, 3);
    Vector3d updated_ba = BAccBias_ + delta.segment(18, 3);

    this->concatStates(updated_q, updated_v, updated_p, updated_lfp, updated_rfp, updated_bg, updated_ba, x);
}

void QuatEKF::update() {
    this->seprateStates(x_);
    Hd_ = MatrixXd::Zero(measurmentDim_, statesErrDim_);
    y_ = MatrixXd::Zero(measurmentDim_, 1);
    Rd_ = MatrixXd::Zero(measurmentDim_, measurmentDim_);

    if(contact_[0] == 1){
        y_.segment(0, GLeftFootPos_.size()) = BLFootMeasured_ - GBaseRot_.transpose() * (GLeftFootPos_ - GBasePos_);
        Hd_.block(0, 0, 3, 3) = skewSym(GBaseRot_.transpose() * (GLeftFootPos_ - GBasePos_));
        Hd_.block(0, 3, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 6, 3, 3) = -GBaseRot_.transpose();
        Hd_.block(0, 9, 3, 3) = GBaseRot_.transpose();
        Hd_.block(0, 12, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 15, 3, 3) = Matrix3d::Zero();
        Hd_.block(0, 18, 3, 3) = Matrix3d::Zero();
        Rd_.block(0, 0, 3, 3) = measurementNoiseStd_ * Matrix3d::Identity();
    }
    if(contact_[1] == 1){
        y_.segment(3, GRightFootPos_.size()) = BRFootMeasured_ - GBaseRot_.transpose() * (GRightFootPos_ - GBasePos_);
        Hd_.block(3, 0, 3, 3) = skewSym(GBaseRot_.transpose() * (GRightFootPos_ - GBasePos_));
        Hd_.block(3, 3, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 6, 3, 3) = -GBaseRot_.transpose();
        Hd_.block(3, 9, 3, 3) = GBaseRot_.transpose();
        Hd_.block(3, 12, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 15, 3, 3) = Matrix3d::Zero();
        Hd_.block(3, 18, 3, 3) = Matrix3d::Zero();
        Rd_.block(3, 3, 3, 3) = measurementNoiseStd_ * Matrix3d::Identity();
    }
    Sd_ = Hd_ * P_ * Hd_.transpose() + Rd_;
    Kd_ = P_ * Hd_.transpose() * Sd_.inverse();
    deltaX_ = Kd_ * y_;
    this->updateState(deltaX_, x_);
    MatrixXd I = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    P_ = (I - Kd_ * Hd_) * P_ * (I - Kd_ * Hd_).transpose() + Kd_ * Rd_ * Kd_.transpose();
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