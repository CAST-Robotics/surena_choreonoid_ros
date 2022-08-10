#include "LieEKF.h"

LieEKF::LieEKF(){
    Gravity_ = Vector3d(0, 0, -9.81);
    
    BGyro_ = Vector3d::Zero();
    BAcc_ = Vector3d::Zero();

    BLFootMeasured_ = Vector3d::Zero();
    BRFootMeasured_ = Vector3d::Zero();
    BLFootRotMeasured_ = Matrix3d::Identity();
    BRFootRotMeasured_ = Matrix3d::Identity();

    statesDim_ = 3 + 1 + 1 + 1 + 1; // rot, vel, pos, left-contact, right-contact
    thetaDim_ = BGyroBias_.size() + BAccBias_.size();

    statesErrDim_ = 3 + GBaseVel_.size() + GBasePos_.size() + GLeftFootPos_.size() + 
                    GRightFootPos_.size() + BGyroBias_.size() + BAccBias_.size();

    x_ = MatrixXd::Zero(statesDim_, statesDim_);
    xPrev_ = MatrixXd::Zero(statesDim_, statesDim_);
    xPred_ = MatrixXd::Zero(statesDim_, statesDim_);
    theta_ = VectorXd::Zero(thetaDim_);
    thetaPrev_ = VectorXd::Zero(thetaDim_);
    thetaPred_ = VectorXd::Zero(thetaDim_);

    // default initialize states
    Matrix3d R = Matrix3d::Identity();
    Vector3d base_vel = Vector3d::Zero();
    Vector3d base_pos = Vector3d(0, 0, 0.71);
    Vector3d lf_pos = Vector3d(0, 0.1, 0);
    Vector3d rf_pos = Vector3d(0, -0.1, 0);
    Vector3d gyro_bias = Vector3d::Zero();
    Vector3d acc_bias = Vector3d::Zero();

    this->initializeStates(R, base_vel, base_pos, lf_pos, rf_pos, gyro_bias, acc_bias);
    this->concatStates(R, base_vel, base_pos, lf_pos, rf_pos, gyro_bias, acc_bias, xPrev_, thetaPrev_);

    this->setDt(0.002);

    P_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    Phi_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);
    // default initial covaricance matrix
    this->initializeCovariance(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

    // default process and measurement noise
    this->setNoiseStd(0.05, 0.05, 0.1, 0.1, 0.1, 0.01);

    updateEnabled_ = true;
}

LieEKF::~LieEKF(){}

void LieEKF::initializeStates(Matrix3d R, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                              Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias){
    GBaseRot_ = R;
    GBaseVel_ = base_vel;
    GBasePos_ = base_pos;
    GLeftFootPos_ = lf_pos;
    GRightFootPos_ = rf_pos;
    BGyroBias_ = gyro_bias;
    BAccBias_ = acc_bias;
}

void LieEKF::concatStates(Matrix3d R, Vector3d base_vel, Vector3d base_pos, Vector3d lf_pos, 
                          Vector3d rf_pos, Vector3d gyro_bias, Vector3d acc_bias, MatrixXd &output, VectorXd &output_theta){
    output = MatrixXd::Identity();
    output.block(0, 0, 3, 3) = R;
    output.block(0, 3, 3, 1) = base_vel;
    output.block(0, 4, 3, 1) = base_pos;
    output.block(0, 5, 3, 1) = lf_pos;
    output.block(0, 6, 3, 1) = rf_pos;

    output_theta.segment(0, 3) = acc_bias;
    output_theta.segment(3, 3) = gyro_bias;
}

void LieEKF::initializeCovariance(double quat_std, double vel_std, double pos_std,
                                  double contact_std, double gyro_std, double acc_std){
    quatStd_ = quat_std;
    velStd_ = vel_std;
    posStd_ = pos_std;
    contactStd_ = contact_std;
    gyroBiasStd_ = gyro_std;
    accBiasStd_ = acc_std;

    P_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);

    P_.block(0, 0, 3, 3) = pow(quatStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(3, 3, 3, 3) = pow(velStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(6, 6, 3, 3) = pow(posStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(9, 9, 3, 3) = pow(contactStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(12, 12, 3, 3) = pow(contactStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(15, 15, 3, 3) = pow(gyroBiasStd_, 2) * MatrixXd::Identity(3, 3);
    P_.block(18, 18, 3, 3) = pow(accBiasStd_, 2) * MatrixXd::Identity(3, 3);
}

void LieEKF::setNoiseStd(double gyro_noise, double acc_noise, double gyro_bias_noise, 
                         double acc_bias_noise, double contact_noise, double measurement_noise){
    gyroNoiseStd_ = gyro_noise;
    accNoiseStd_ = acc_noise;
    gyroBiasNoiseStd_ = gyro_bias_noise;
    accBiasNoiseStd_ = acc_bias_noise;
    contactNoiseStd_ = contact_noise;
    measurementNoiseStd_ = measurement_noise;
}

void LieEKF::setSensorData(Vector3d gyro, Vector3d acc){
    BGyro_ = gyro;
    BAcc_ = acc;
}

void LieEKF::setMeasuredData(Vector3d l_kynematic, Vector3d r_kynematic, Matrix3d l_kynematic_rot, Matrix3d r_kynematic_rot){
    BLFootMeasured_ = l_kynematic;
    BRFootMeasured_ = r_kynematic;
    
    BLFootRotMeasured_ = l_kynematic_rot;
    BRFootRotMeasured_ = r_kynematic_rot;
}

void LieEKF::seprateStates(const MatrixXd &x, const VectorXd &theta){

    GBaseRot_ = x.block(0, 0, 3, 3);
    GBaseVel_ = x.block(0, 3, 3, 1);
    GBasePos_ = x.block(0, 4, 3, 1);
    GLeftFootPos_ = x.block(0, 5, 3, 1);
    GRightFootPos_ = x.block(0, 6, 3, 1);

    BAccBias_ = theta.segment(0, 3);
    BGyroBias_ = theta.segment(3, 3);
}

void LieEKF::predict(){
    
    this->seprateStates(xPrev_, thetaPrev_);
    // remove bias from IMU data
    BAcc_ = BAcc_ - BAccBias_;
    BGyro_ = BGyro_ - BGyroBias_;

    Matrix3d acc_skew = skewSym(BAcc_);
    
    Matrix3d gamma_0 = gamma(dt_ * BGyro_, 0);
    Matrix3d gamma_1 = gamma(dt_ * BGyro_, 1);
    Matrix3d gamma_2 = gamma(dt_ * BGyro_, 2);

    // predict rotation matrix
    Matrix3d predicted_R = GBaseRot_ * gamma_0;

    // predict position and velocity
    Vector3d predicted_v = GBaseVel_ + dt_ * (GBaseRot_ * gamma_1 * BAcc_ + Gravity_);
    Vector3d predicted_p = GBasePos_ + dt_ * GBaseVel_ + pow(dt_, 2) * (GBaseRot_ * gamma_2 * BAcc_ + 0.5 * Gravity_);

    // predict foot positions
    // what happens if the measurement is not available?
    Vector3d predicted_lf_p;
    Vector3d predicted_rf_p;
    predicted_lf_p = predicted_p + predicted_R * BLFootMeasured_;
    predicted_rf_p = predicted_p + predicted_R * BRFootMeasured_;
    
    predicted_lf_p = contact_[0] * GLeftFootPos_ + (1 - contact_[0]) * predicted_lf_p;
    predicted_rf_p = contact_[1] * GRightFootPos_ + (1 - contact_[1]) * predicted_rf_p;
    
    // predict biases
    Vector3d predicted_gyro_bias = BGyroBias_;
    Vector3d predicted_acc_bias = BAccBias_;

    Matrix3d g_skew = skewSym(Gravity_);

    Matrix3d gamma_0 = gamma(dt_ * BGyro_, 0);
    Matrix3d gamma_1 = gamma(dt_ * BGyro_, 1);
    Matrix3d gamma_2 = gamma(dt_ * BGyro_, 2);
    
    Matrix3d phi_skew = skewSym(dt_ * BGyro_);
    Matrix3d acc_skew = skewSym(BAcc_);
    double phi_norm = (dt_ * BGyro_).norm();
    Matrix3d psi_1;
    Matrix3d psi_2;

    if(BGyro_.norm() > 1e-6){

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
    
    Phi_ = MatrixXd::Identity(statesErrDim_, statesErrDim_);

    Phi_.block(3, 0, 3, 3) = dt_ * g_skew;
    Phi_.block(6, 0, 3, 3) = 0.5 * pow(dt_, 2) * g_skew;
    Phi_.block(6, 3, 3, 3) = dt_ * Matrix3d::Identity();
    Phi_.block(0, 15, 3, 3) = -dt_ * GBaseRot_ * gamma_1;
    Phi_.block(3, 15, 3, 3) = -dt_ * skewSym(predicted_v) * GBaseRot_ * gamma_1 + pow(dt_, 2) * GBaseRot_ * psi_1;
    Phi_.block(6, 15, 3, 3) = -dt_ * skewSym(predicted_p) * GBaseRot_ * gamma_1 + pow(dt_, 3) * GBaseRot_ * psi_2;
    Phi_.block(9, 15, 3, 3) = -dt_ * skewSym(predicted_lf_p) * GBaseRot_ * gamma_1;
    Phi_.block(12, 15, 3, 3) = -dt_ * skewSym(predicted_rf_p) * GBaseRot_ * gamma_1;
    Phi_.block(3, 18, 3, 3) = -dt_ * GBaseRot_ * gamma_1;
    Phi_.block(6, 18, 3, 3) = -pow(dt_, 2) * GBaseRot_ * gamma_2;

    this->updateQd();

    this->concatStates(predicted_R, predicted_v, predicted_p, predicted_lf_p, predicted_rf_p, predicted_gyro_bias, predicted_acc_bias, xPred_, thetaPred_);
    P_ = Phi_ * P_ * Phi_.transpose() + Qd_;
}

void LieEKF::updateQd() {
    Matrix3d hR_L = BLFootRotMeasured_;
    Matrix3d hR_R = BRFootRotMeasured_;
    Qd_ = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    Qd_.block(0, 0, 3, 3) = pow(gyroNoiseStd_, 2) * Matrix3d::Identity();
    Qd_.block(3, 3, 3, 3) = pow(accNoiseStd_, 2) * Matrix3d::Identity();
    Qd_.block(6, 6, 3, 3) = Matrix3d::Zero();
    Qd_.block(9, 9, 3, 3) = hR_L * (pow(contactNoiseStd_, 2) * Matrix3d::Identity() + (1e4 * (1 - contact_[0])) * Matrix3d::Identity()) * hR_L.transpose();
    Qd_.block(12, 12, 3, 3) = hR_R * (pow(contactNoiseStd_, 2) * Matrix3d::Identity() + (1e4 * (1 - contact_[1])) * Matrix3d::Identity()) * hR_R.transpose();
    Qd_.block(15, 15, 3, 3) = pow(gyroBiasNoiseStd_, 2) * Matrix3d::Identity();
    Qd_.block(18, 18, 3, 3) = pow(accBiasNoiseStd_, 2) * Matrix3d::Identity();

    MatrixXd G = MatrixXd::Zero(statesErrDim_, statesErrDim_);
    G.block(0, 0, 15, 15) = adjoint(xPrev_); // adjoint of xPrev_
    G.block(15, 15, 6, 6) = MatrixXd::Identity(6, 6);

    Qd_ = dt_ * Phi_ * G * Qd_ * G.transpose() * Phi_.transpose();
}