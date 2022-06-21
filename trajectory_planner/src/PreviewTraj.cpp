#include "PreviewTraj.h"

PreviewTraj::PreviewTraj(double robot_height, int n=320) : robotHeight_(robot_height), N_(n) {
    
    dt_ = 0.005;
    g_ = 9.81;

    A_ << 1, dt_, pow(dt_, 2) / 2,
         0, 1, dt_,
         0, 0, 1;
    b_ << pow(dt_, 3) / 6,
         pow(dt_, 2) / 2,
         dt_;
    c_ << 1, 0, -(robotHeight_ / g_);
    Q_ = 1 * MatrixXd::Identity(1, 1);
    R_ = pow(10, -6) * MatrixXd::Identity(1, 1);

    x0_ << 0, 0, 0;
    P_ = MatrixXd::Zero(4, 4);
    Gd_ = VectorXd::Zero(N_);

    error_ = 0;

    ZMP_ = VectorXd::Zero(800 + N_);
    x_ = new Vector3d[800];
    x_[0] = x0_;
    for(int i=200; i < 400; i++) {
        ZMP_[i] = 1.0;
    }
    for(int i=400; i < 600; i++) {
        ZMP_[i] = -1.0;
    }
    for(int i=600; i < 800 + N_; i++) {
        ZMP_[i] = 1.0;
    }
}

void PreviewTraj::setDt(double dt){
    dt_ = dt;
}

void PreviewTraj::computeWeight(){

    MatrixXd A_bar = MatrixXd::Zero(4, 4);
    A_bar(0, 0) = 1.0;
    A_bar.block(0, 1, 1, 3) = c_ * A_;
    A_bar.block(1, 1, 3, 3) = A_;

    MatrixXd B_bar = MatrixXd::Zero(4, 1);
    B_bar.block(0, 0, 1, 1) = c_ * b_;
    B_bar.block(1, 0, 3, 1) = b_;

    MatrixXd Q_bar = MatrixXd::Zero(4, 4);
    Q_bar.block(0, 0, 1, 1) = Q_;

    solveRiccatiIterationD(A_bar, B_bar, Q_bar, R_, P_);

    MatrixXd I_bar = MatrixXd::Zero(4, 1);
    I_bar(0, 0) = 1.0;

    MatrixXd F_bar = MatrixXd::Zero(4, 3);
    F_bar.block(0, 0, 1, 3) = c_ * A_;
    F_bar.block(1, 0, 3, 3) = A_;

    Gl_ = (R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * P_ * I_bar;
    Gx_ = (R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * P_ * F_bar;

    MatrixXd Ac_bar = A_bar - B_bar * (R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * P_ * A_bar;
    MatrixXd X_bar = -Ac_bar.transpose() * P_ * I_bar;
    
    Gd_(0) = -Gl_(0);
    for(int i=1; i < N_; i++) {
        Gd_(i) = ((R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * X_bar)(0);
        X_bar = Ac_bar.transpose() * X_bar;
    }

    // K_ = (R_ + b_.transpose() * P_ * b_).inverse() * b_.transpose() * P_ * A_;
    // for(int i=0; i<N_; i++){
    //     Matrix3d Ab = (A_ - b_ * K_).transpose();
    //     EigenSolver<Matrix3d> es(Ab);
    //     Matrix3d D = es.pseudoEigenvalueMatrix();
    //     Matrix3d V = es.pseudoEigenvectors();
    //     D << pow(D(0, 0), dt_ * i), 0, 0,
    //          0, pow(D(1, 1), dt_ * i), 0,
    //          0, 0, pow(D(2, 2), dt_ * i);
    //     Ab = V * D * V.inverse();
    //     f_[i] = ((R_ + b_.transpose() * P_ * b_).inverse() * b_.transpose() * (Ab) * c_.transpose() * Q_)(0);
    // }
}

void PreviewTraj::computeTraj(){

    for(int i=1; i<800; i++){
        error_ += c_ * x_[i - 1] - ZMP_[i - 1];
        double uk = ((-Gl_ * error_ - Gx_ * x_[i - 1]) - Gd_.transpose() * ZMP_.block(i, 0, N_, 1))(0);
        x_[i] = A_ * x_[i - 1] + uk * b_;
        //x_[i](2) = c_ * x_[i - 1];
        double p = c_ * x_[i - 1];
        cout << x_[i](0) << ", " << x_[i](1) << ", " << x_[i](2) << ", " << p << ", " << ZMP_[i] << endl;
    }
}

