#include "PreviewTraj.h"

PreviewTraj::PreviewTraj(double robot_height, int n, double dt) : robotHeight_(robot_height), N_(n), dt_(dt) {
    
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

    error_ = Vector3d(0, 0, 0);

    string config_path = ros::package::getPath("trajectory_planner") + "/config/config.yaml";
    ZMPPlanner_ = new ZMPPlanner(true, dt_);
    ZMPPlanner_->setConfigPath(config_path);
    // MatrixXd a{{1, 4, 4, 3, -2, 0}};
    // MatrixXd c = MatrixXd::Zero(1, 6);
    // VectorXd b(6);
    // b << 0, 1, 2, 3, 4, 5;
    // MatrixXd q;
    // ZMPPlanner_->cubicPolyTraj(a, b, 0.005, c, q);
    ZMPPlanner_->planInitialDSPZMP();
    ZMPPlanner_->planStepsZMP();
    ZMPPlanner_->planFinalDSPZMP();
    int traj_size = ZMPPlanner_->getTrajSize();
    // AnkleTraj a(true, dt_);
    // a.setConfigPath(config_path);
    // a.planSteps();
    // cout << traj_size << endl;
    x_ = new Vector3d[traj_size];
    y_ = new Vector3d[traj_size];
    x_[0] = x0_;
    y_[0] = x0_;
    com_pos.push_back(Vector3d(0, 0, robotHeight_));
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
}

void PreviewTraj::computeTraj(){

    for(int i=1; i<ZMPPlanner_->getTrajSize(); i++){

        error_(0) += c_ * x_[i - 1] - ZMPPlanner_->getZMP(i - 1)(0);
        error_(1) += c_ * y_[i - 1] - ZMPPlanner_->getZMP(i - 1)(1);

        Vector3d preview_u(0, 0, 0);
        for(int j=0; j<N_; j++){
            preview_u += double(Gd_(j)) * (ZMPPlanner_->getZMP(i + j));
        }
        
        double ux = ((-Gl_ * error_(0) - Gx_ * x_[i - 1])(0) - preview_u(0));
        double uy = ((-Gl_ * error_(1) - Gx_ * y_[i - 1])(0) - preview_u(1));
        x_[i] = A_ * x_[i - 1] + ux * b_;
        y_[i] = A_ * y_[i - 1] + uy * b_;

        // double p = c_ * x_[i - 1];
        // cout << x_[i](0) << ", " << x_[i](1) << ", " << x_[i](2) << ", " << p << ", ";
        // p = c_ * y_[i - 1];
        // cout << y_[i](0) << ", " << y_[i](1) << ", " << y_[i](2) << ", " << p << endl;

        com_pos.push_back(Vector3d(x_[i](0), y_[i](0), robotHeight_));
    }
}

vector<Vector3d> PreviewTraj::getCOM(){
    return com_pos;
}
