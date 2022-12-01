#include "Robot.h" 
//#include <chrono>
//using namespace std::chrono;

void write2File(Vector3d* input, int size, string file_name="data"){
    ofstream output_file(file_name + ".csv");
    for(int i=0; i<size; i++){
         output_file << input[i](0) << " ,";
         output_file << input[i](1) << " ,";
         output_file << input[i](2) << " ,";
         output_file << "\n";
    }
    output_file.close();
}


Robot::Robot(ros::NodeHandle *nh, Controller robot_ctrl){

    trajNode_.setCallbackQueue(&trajCbQueue_);
    trajGenServer_ = trajNode_.advertiseService("/traj_gen", 
            &Robot::trajGenCallback, this);
    std::thread trajThread([this](){
        ros::SingleThreadedSpinner traj_spinner;
        traj_spinner.spin(&this->trajCbQueue_);
    });
    trajThread.detach();
    
    jntAngsServer_ = nh->advertiseService("/jnt_angs", 
            &Robot::jntAngsCallback, this);
    generalTrajServer_ = nh->advertiseService("/general_traj", 
            &Robot::generalTrajCallback, this);
    resetTrajServer_ = nh->advertiseService("/reset_traj",
            &Robot::resetTrajCallback, this);

    baseOdomPub_ = nh->advertise<nav_msgs::Odometry>("/odom", 50);
    
    // SURENA IV geometrical params
    
    thigh_ = 0.36;  // SR1: 0.3535, Surena4: 0.37, Surena5: 0.36
    shank_ = 0.35;     // SR1: 0.3, Surena4: 0.36, Surena5: 0.35
    torso_ = 0.1;    // SR1: 0.09, Surena4: 0.115, Surena5: 0.1

    mass_ = 48.3; // SR1: ?, Surena4: 48.3, Surena5: ?

    dataSize_ = 0;
    rSole_ << 0.0, -torso_, 0.0;
    lSole_ << 0.0, torso_, 0.0;       // might be better if these two are input argument of constructor
    isTrajAvailable_ = false;

    Vector3d a[12];
	Vector3d b[12];
	// Defining Joint Axis
	a[0] << 0.0, 0.0, 1.0;
	a[1] << 1.0, 0.0, 0.0;
	a[2] << 0.0, 1.0, 0.0;
	a[3] << 0.0, 1.0, 0.0;
	a[4] << 0.0, 1.0, 0.0;
	a[5] << 1.0, 0.0, 0.0;
	a[6] = a[0];
	a[7] = a[1];
	a[8] = a[2];
	a[9] = a[3];
	a[10] = a[4];
	a[11] = a[5];
	// Defining Joint Positions
	b[0] << 0.0, -torso_, 0.0;
	b[1] << 0.0, 0.0, 0.0;
	b[2] = b[1];
	b[3] << 0.0, 0.0, -thigh_;
	b[4] << 0.0, 0.0, -shank_;
	b[5] = b[1];
	b[6] = -b[0];
	b[7] = b[1];
	b[8] = b[2];
	b[9] = b[3];
	b[10] = b[4];
	b[11] = b[5];

    _Link* pelvis = new _Link(0, Vector3d::Ones(3), Vector3d::Ones(3), 3.0, Matrix3d::Identity(3,3));
    Vector3d position(0.0, 0.0, 0.0);
	pelvis->initPose(position, Matrix3d::Identity(3, 3));
    links_[0] = pelvis;
    _Link* rHipY = new _Link(1, a[0], b[0], 3.0, Matrix3d::Identity(3, 3), links_[0]);
    links_[1] = rHipY;
    _Link* rHipR = new _Link(2, a[1], b[1], 3.0, Matrix3d::Identity(3, 3), links_[1]);
    links_[2] = rHipR;
    _Link* rHipP = new _Link(3, a[2], b[2], 3.0, Matrix3d::Identity(3, 3), links_[2]);
    links_[3] = rHipP;
    _Link* rKnee = new _Link(4, a[3], b[3], 3.0, Matrix3d::Identity(3, 3), links_[3]);
    links_[4] = rKnee;
    _Link* rAnkleP = new _Link(5, a[4], b[4], 3.0, Matrix3d::Identity(3, 3), links_[4]);
    links_[5] = rAnkleP; 
    _Link* rAnkleR = new _Link(6, a[5], b[5], 3.0, Matrix3d::Identity(3, 3), links_[5]);
    links_[6] = rAnkleR; 

    _Link* lHipY = new _Link(7, a[6], b[6], 3.0, Matrix3d::Identity(3, 3), links_[0]);
    links_[7] = lHipY;
    _Link* lHipR = new _Link(8, a[7], b[7], 3.0, Matrix3d::Identity(3, 3), links_[7]);
    links_[8] = lHipR;
    _Link* lHipP = new _Link(9, a[8], b[8], 3.0, Matrix3d::Identity(3, 3), links_[8]);
    links_[9] = lHipP;
    _Link* lKnee = new _Link(10, a[9], b[9], 3.0, Matrix3d::Identity(3, 3), links_[9]);
    links_[10] = lKnee;
    _Link* lAnkleP = new _Link(11, a[10], b[10], 3.0, Matrix3d::Identity(3, 3), links_[10]);
    links_[11] = lAnkleP; 
    _Link* lAnkleR = new _Link(12, a[11], b[11], 3.0, Matrix3d::Identity(3, 3), links_[11]);
    links_[12] = lAnkleR;

    onlineWalk_ = robot_ctrl;
    quatEKF_ = new QuatEKF();
    lieEKF_ = new LieEKF();
    // stepPlanner_ = new FootStepPlanner(torso_);
    // ZMPPlanner_ = new ZMPPlanner();

    //cout << "Robot Object has been Created" << endl;
}

void Robot::spinOnline(int iter, double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer, double* joint_angles){
    // update joint positions
    for (int i = 0; i < 13; i ++){
        links_[i]->update(config[i], jnt_vel[i], 0.0);
    }
    // Do the Forward Kinematics for Lower Limb
    links_[12]->FK();
    links_[6]->FK();    // update all raw values of sensors and link states
    //updateState(config, torque_r, torque_l, f_r, f_l, gyro, accelerometer);
    // int contact[2];
    // if(robotState_[iter] == 2){
    //     contact[0] = 0;
    //     contact[1] = 1;
    // }else if(robotState_[iter] == 3){
    //     contact[0] = 1;
    //     contact[1] = 0;
    // }else{
    //     contact[0] = 1;
    //     contact[1] = 1;
    // }
    // add noise to the sensor values
    // std::default_random_engine generator;
    // generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    // std::normal_distribution<double> dist(0.0, 0.05);
    // std::normal_distribution<double> dist1(0.0, 0.15);
    // quatEKF_->setDt(dt_);
    // lieEKF_->setDt(dt_);
    // //quatEKF_->runFilter(gyro + Vector3d(dist(generator), dist(generator), dist(generator)), accelerometer + Vector3d(dist(generator), dist(generator), dist(generator)), links_[12]->getPose(), links_[6]->getPose(), links_[12]->getRot(), links_[6]->getRot(), contact, true);
    // lieEKF_->runFilter(gyro + Vector3d(dist(generator), dist(generator), dist(generator)), accelerometer + Vector3d(dist1(generator), dist1(generator), dist1(generator)), links_[12]->getPose(), links_[6]->getPose(), links_[12]->getRot(), links_[6]->getRot(), contact, true);
    //baseOdomPublisher(lieEKF_->getGBasePose(), lieEKF_->getGBaseVel(), lieEKF_->getGBaseQuat());
    //baseOdomPublisher(quatEKF_->getGBasePose(), quatEKF_->getGBaseVel(), quatEKF_->getGBaseQuat());
    // cout << gyro(0) << ',' << gyro(1) << ',' << gyro(2) << ",";
    // cout << accelerometer(0) << ',' << accelerometer(1) << ',' << accelerometer(2) << ",";
    // cout << links_[6]->getPose()(0) << ',' << links_[6]->getPose()(1) << ',' << links_[6]->getPose()(2) << ",";
    // cout << links_[12]->getPose()(0) << ',' << links_[12]->getPose()(1) << ',' << links_[12]->getPose()(2) << ",";
    // Quaterniond rankle_quat(links_[6]->getRot());
    // Quaterniond lankle_quat(links_[12]->getRot());
    // cout << rankle_quat.w() << ',' << rankle_quat.x() << ',' << rankle_quat.y() << ',' << rankle_quat.z() << ",";
    // cout << lankle_quat.w() << ',' << lankle_quat.x() << ',' << lankle_quat.y() << ',' << lankle_quat.z() << ",";
    // if(robotState_[iter] == 2){
    //     cout << 1 << "," << 0 << endl;
    // }else if(robotState_[iter] == 3){
    //     cout << 0 << "," << 1 << endl;
    // }else{
    //     cout << 1 << "," << 1 << endl;
    // }
    MatrixXd lfoot(3,1);
    MatrixXd rfoot(3,1);
    Matrix3d attitude = MatrixXd::Identity(3,3);
    MatrixXd pelvis(3,1);
    // pelvis << CoMPos_[iter](0), CoMPos_[iter](1), CoMPos_[iter](2);
    // lfoot << lAnklePos_[iter](0), lAnklePos_[iter](1), lAnklePos_[iter](2);
    // rfoot << rAnklePos_[iter](0), rAnklePos_[iter](1), rAnklePos_[iter](2);

    pelvis << CoMPos_[iter](0), CoMPos_[iter](1), CoMPos_[iter](2);
    lfoot << lAnklePos_[iter](0), lAnklePos_[iter](1), lAnklePos_[iter](2);
    rfoot << rAnklePos_[iter](0), rAnklePos_[iter](1), rAnklePos_[iter](2);
    // cout << CoMPos_[iter](0) << ", " << CoMPos_[iter](1) << ", " << CoMPos_[iter](2) << ", ";
    // cout << lAnklePos_[iter](0) << ", " << lAnklePos_[iter](1) << ", " << lAnklePos_[iter](2) << ", ";
    // cout << rAnklePos_[iter](0) << ", " << rAnklePos_[iter](1) << ", " << rAnklePos_[iter](2) << endl;
    // int traj_index = findTrajIndex(trajSizes_, trajSizes_.size(), iter);
/*
    if(iter > trajSizes_[0] && iter < trajSizes_[1]){
        Vector3d r_wrench;
        Vector3d l_wrench;
        distributeFT(zmpd_[iter - trajSizes_[0]], rAnklePos_[iter], lAnklePos_[iter], r_wrench, l_wrench);
        //cout << r_wrench(0) << "," << r_wrench(1) << "," << r_wrench(2) << ","
        //<< l_wrench(0) << "," << l_wrench(1) << "," << l_wrench(2) << endl;
        double delta_z = onlineWalk_.footLenController(r_wrench(0) - l_wrench(0), f_r - f_l, 0.02, 1500);
        lfoot << lAnklePos_[iter](0), lAnklePos_[iter](1), lAnklePos_[iter](2) - 0.5 * delta_z;
        rfoot << rAnklePos_[iter](0), rAnklePos_[iter](1), rAnklePos_[iter](2) + 0.5 * delta_z;
        cout << lAnklePos_[iter](2) << ',' << rAnklePos_[iter](2) << endl;
        //cout << r_wrench(0) << ',' << l_wrench(0) << ',' << f_r << ',' << f_l << endl;
        //cout<< delta_z <<endl;
    }
    else{
        lfoot << lAnklePos_[iter](0), lAnklePos_[iter](1), lAnklePos_[iter](2);
        rfoot << rAnklePos_[iter](0), rAnklePos_[iter](1), rAnklePos_[iter](2);

    }
*/
    // if(trajContFlags_[traj_index] == true){
    //     Vector3d zmp_ref = onlineWalk_.dcmController(xiDesired_[iter+1], xiDot_[iter+1], realXi_[iter], COM_height_);
    //     Vector3d cont_out = onlineWalk_.comController(CoMPos_[iter], CoMDot_[iter+1], FKCoM_[iter], zmp_ref, realZMP_[iter]);
    //     pelvis = cont_out;
    // }
    //cout << CoMRot_[iter].eulerAngles(0, 1, 2)(0) << "," << CoMRot_[iter].eulerAngles(0, 1, 2)(1) << "," << CoMRot_[iter].eulerAngles(0, 1, 2)(2) << "," <<
    //        lAnkleRot_[iter].eulerAngles(0, 1, 2)(0) << "," << lAnkleRot_[iter].eulerAngles(0, 1, 2)(1) << "," << lAnkleRot_[iter].eulerAngles(0, 1, 2)(2) << "," <<
    //        rAnkleRot_[iter].eulerAngles(0, 1, 2)(0) << "," << rAnkleRot_[iter].eulerAngles(0, 1, 2)(1) << "," << rAnkleRot_[iter].eulerAngles(0, 1, 2)(2) << endl;
    //doIK(pelvis, CoMRot_[iter], lfoot, lAnkleRot_[iter], rfoot, rAnkleRot_[iter]);
    doIK(pelvis, CoMRot_[iter], lfoot, lAnkleRot_[iter], rfoot, rAnkleRot_[iter]);

    for(int i = 0; i < 12; i++)
        joint_angles[i] = joints_[i];     // right leg(0-5) & left leg(6-11)
}

void Robot::updateState(double config[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer){
    
    // Interpret IMU data
    Vector3d change_attitude;
    Vector3d base_attitude = links_[0]->getRot().eulerAngles(0, 1, 2);
    //cout << base_attitude(0) * 180/M_PI << ", " << base_attitude(1) * 180/M_PI << ", " << base_attitude(2) * 180/M_PI << endl;
    
    change_attitude[0] = gyro(0) + tan(base_attitude(1)) * (gyro(1) * sin(base_attitude(0)) + gyro(2) * cos(base_attitude(0)));
    change_attitude[1] = gyro(1) * cos(base_attitude(0)) - gyro(2) * sin(base_attitude(0));
    change_attitude[2] = 1/cos(base_attitude(1)) * (gyro(1) * sin(base_attitude(0)) + gyro(2) * cos(base_attitude(0)));

    base_attitude += this->dt_ * change_attitude;
    Matrix3d rot = (AngleAxisd(base_attitude[0], Vector3d::UnitX())
                  * AngleAxisd(base_attitude[1], Vector3d::UnitY())
                  * AngleAxisd(base_attitude[2], Vector3d::UnitZ())).matrix();
    // links_[0]->initPose(Vector3d::Zero(3), rot);

    // Update swing/stance foot
    if (links_[12]->getPose()(2) < links_[6]->getPose()(2)){
        rightSwings_ = true;
        leftSwings_ = false;
    }
    else if (links_[6]->getPose()(2) < links_[12]->getPose()(2)){
        rightSwings_ = false;
        leftSwings_ = true;
    }
    else{
        rightSwings_ = false;
        leftSwings_ = false;
    }

    // cout << links_[6]->getPose()(0) << ", " << links_[6]->getPose()(1) << ", " << links_[6]->getPose()(2) << ", ";
    // cout << links_[12]->getPose()(0) << ", " << links_[12]->getPose()(1) << ", " << links_[12]->getPose()(2) << ", ";
    
    // Update CoM and Sole Positions
    updateSolePosition();

    // Calculate ZMP with FT data
    Vector3d l_zmp = getZMPLocal(torque_l, f_l);
    Vector3d r_zmp = getZMPLocal(torque_r, f_r);

    if (abs(f_l) < 5)
        f_l = 0;
    if (abs(f_r) < 5)
        f_r = 0;

    realZMP_[index_] = ZMPGlobal(rSole_ + l_zmp, lSole_ + r_zmp, f_r, f_l);

}

void Robot::updateSolePosition(){

    Matrix3d r_dot = this->rDot_(links_[0]->getRot());

    if(leftSwings_ && (!rightSwings_)){
        lSole_ = rSole_ - links_[6]->getPose() + links_[12]->getPose();
        FKCoM_[index_] = lSole_ - links_[0]->getRot() * links_[12]->getPose();
        //FKCoMDot_[index_] = - links_[6]->getVel().block<3,1>(0, 0);
        //FKCoMDot_[index_] = -links_[0]->getRot() * links_[6]->getVel().block<3,1>(0, 0) - r_dot * links_[6]->getPose();
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    else if ((!leftSwings_) && rightSwings_){
        Matrix<double, 6, 1> q_dot;
        rSole_ = lSole_ - links_[12]->getPose() + links_[6]->getPose();
        FKCoM_[index_] = rSole_ - links_[0]->getRot() * links_[6]->getPose();
        //FKCoMDot_[index_] = - links_[12]->getVel().block<3,1>(0, 0);
        //FKCoMDot_[index_] = -links_[0]->getRot() * links_[12]->getVel().block<3,1>(0, 0) - r_dot * links_[12]->getPose();
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    else{   // double support
        FKCoM_[index_] = rSole_ - links_[0]->getRot() * links_[6]->getPose();
        //FKCoMDot_[index_] = - links_[6]->getVel().block<3,1>(0, 0);
        //FKCoMDot_[index_] = -links_[0]->getRot() * links_[6]->getVel().block<3,1>(0, 0) - r_dot * links_[6]->getPose();
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }

    // 3-point backward formula for numeraical differentiation: 
    // https://www3.nd.edu/~zxu2/acms40390F15/Lec-4.1.pdf
    Vector3d f1, f0;
    if (index_ == 0) {f1 = Vector3d::Zero(3); f0 = Vector3d::Zero(3);}
    else if (index_ == 1) {f1 = FKCoM_[index_-1]; f0 = Vector3d::Zero(3);}
    else {f1 = FKCoM_[index_-1]; f0 = FKCoM_[index_-2];} 
    FKCoMDot_[index_] = (f0 - 4 * f1 + 3 * FKCoM_[index_])/(2 * this->dt_);
    realXi_[index_] = FKCoM_[index_] + FKCoMDot_[index_] / sqrt(K_G/COM_height_);

}

Vector3d Robot::getZMPLocal(Vector3d torque, double fz){
    // Calculate ZMP for each foot
    Vector3d zmp(0.0, 0.0, 0.0);
    if (fz == 0){
        //ROS_WARN("No Correct Force Value!");
        return zmp;
    }
    zmp(0) = -torque(1)/fz;
    zmp(1) = -torque(0)/fz;
    return zmp;
}

Vector3d Robot::ZMPGlobal(Vector3d zmp_r, Vector3d zmp_l, double f_r, double f_l){
    // Calculate ZMP during Double Support Phase
    Vector3d zmp(0.0, 0.0, 0.0);
    if (f_r + f_l == 0){
        //ROS_WARN("No Foot Contact, Check the Robot!");
        return zmp;
    }
    //assert(!(f_r + f_l == 0));

    return (zmp_r * f_r + zmp_l * f_l) / (f_r + f_l);
}

void Robot::spinOffline(int iter, double* config){
    
    MatrixXd lfoot(3,1);
    MatrixXd rfoot(3,1);
    Matrix3d attitude = MatrixXd::Identity(3,3);
    MatrixXd pelvis(3,1);
    lfoot << lAnklePos_[iter](0), lAnklePos_[iter](1), lAnklePos_[iter](2);
    rfoot << rAnklePos_[iter](0), rAnklePos_[iter](1), rAnklePos_[iter](2);
    pelvis << CoMPos_[iter](0), CoMPos_[iter](1), CoMPos_[iter](2);
    doIK(pelvis,attitude,lfoot,attitude,rfoot,attitude);

    for(int i = 0; i < 12; i++)
        config[i] = joints_[i];     // right leg(0-5) & left leg(6-11)
}

void Robot::doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR){
    // Calculates and sets Robot Leg Configuration at each time step
    double* q_left = this->geometricIK(pelvisP, pelvisR, leftAnkleP, leftAnkleR, true);
    double* q_right = this->geometricIK(pelvisP, pelvisR, rightAnkleP, rightAnkleR, false);
    for(int i = 0; i < 6; i ++){
        joints_[i] = q_right[i];
        joints_[i+6] = q_left[i];
    }
    delete[] q_left;
    delete[] q_right;
}

Matrix3d Robot::Rroll(double phi){
    // helper Function for Geometric IK
    MatrixXd R(3,3);
    double c=cos(phi);
    double s=sin(phi);
    R<<1,0,0,0,c,-1*s,0,s,c;
    return R;
}

Matrix3d Robot::RPitch(double theta){
    // helper Function for Geometric IK
    MatrixXd Ry(3,3);
    double c=cos(theta);
    double s=sin(theta);
    Ry<<c,0,s,0,1,0,-1*s,0,c;
    return Ry;

}

Matrix3d Robot::rDot_(Matrix3d R){
    AngleAxisd angle_axis(R);
    Matrix3d r_dot, temp;
    temp << 0.0, -angle_axis.axis()(2), angle_axis.axis()(1),
             angle_axis.axis()(2), 0.0, -angle_axis.axis()(0),
             -angle_axis.axis()(1), angle_axis.axis()(0), 0.0;
    r_dot = temp * R;
    return r_dot;
}

double* Robot::geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft){
    /*
        Geometric Inverse Kinematic for Robot Leg (Section 2.5  Page 53)
        Reference: Introduction to Humanoid Robotics by Kajita        https://www.springer.com/gp/book/9783642545351
        1 ----> Body        7-----> Foot
    */
   
    double* q = new double[6];  
    MatrixXd D(3,1);

    if (isLeft)
        D << 0.0,torso_,0.0;
    else
        D << 0.0,-torso_,0.0;
        
    MatrixXd r = r7.transpose() * (p1 + r1 * D - p7);
    double C = r.norm();
    double c3 = (pow(C,2) - pow(thigh_,2) - pow(shank_,2))/(2 * thigh_ * shank_);
    if (c3 >= 1){
        q[3] = 0.0;
        // Raise error
    }else if(c3 <= -1){
        q[3] = M_PI;
        // Raise error
    }else{
        q[3] = acos(c3);       // Knee Pitch
    }
    double q4a = asin((thigh_/C) * sin(M_PI - q[3]));
    q[5] = atan2(r(1,0),r(2,0));   //Ankle Roll
    if (q[5] > M_PI/2){
        q[5] = q[5] - M_PI;
        // Raise error
    }
    else if (q[5] < -M_PI / 2){
        q[5] = q[5] + M_PI;
        // Raise error
    }
    int sign_r2 = 1;
    if(r(2,0) < 0)
        sign_r2 = -1;
    q[4] = -atan2(r(0,0),sign_r2 * sqrt(pow(r(1,0),2) + pow(r(2,0),2))) - q4a;      // Ankle Pitch
    Matrix3d R = r1.transpose() * r7 * Rroll(-q[5]) * RPitch(-q[3] - q[4]);
    q[0] = atan2(-R(0,1),R(1,1));         // Hip Yaw
    q[1] = atan2(R(2,1), -R(0,1) * sin(q[0]) + R(1,1) * cos(q[0]));           // Hip Roll
    q[2] = atan2(-R(2,0), R(2,2));        // Hip Pitch
    return q;
}

int Robot::findTrajIndex(vector<int> arr, int n, int K)
{
    /*
        a binary search function to find the index of the running trajectory.
    */
    int start = 0;
    int end = n - 1;
    while (start <= end) {
        int mid = (start + end) / 2;
 
        if (arr[mid] == K)
            return mid + 1;
        else if (arr[mid] < K)
            start = mid + 1;
        else
            end = mid - 1;
    }
    return end + 1;
}

void Robot::distributeFT(Vector3d zmp, Vector3d r_foot,Vector3d l_foot, Vector3d &r_wrench, Vector3d &l_wrench){
    
    double k_f = abs((zmp(1) - r_foot(1))) / abs((r_foot(1) - l_foot(1)));
    l_wrench(0) = -k_f * mass_ * K_G;
    r_wrench(0) = -(1 - k_f) * mass_ * K_G;

    l_wrench(1) = l_wrench(0) * (zmp(1) - l_foot(1));
    r_wrench(1) = r_wrench(0) * (zmp(1) - r_foot(1));

    l_wrench(2) = l_wrench(0) * (zmp(0) - l_foot(0));
    r_wrench(2) = r_wrench(0) * (zmp(0) - r_foot(0));
}


bool Robot::trajGenCallback(trajectory_planner::Trajectory::Request  &req,
                            trajectory_planner::Trajectory::Response &res)
{
    /*
        ROS service for generating robot COM & ankles trajectories
    */
    //ROS_INFO("Generating Trajectory started.");
    //auto start = high_resolution_clock::now();
    // int trajectory_size = int(((req.step_count + 2) * req.t_step) / req.dt);
    // double alpha = req.alpha;
    bool use_file = req.use_file;
    double init_ds = req.t_init_double_support;
    double t_ds = req.t_double_support;
    double t_s = req.t_step;
    double final_ds = req.t_final_double_support;
    COM_height_ = req.COM_height;
    double step_len = req.step_length;
    double step_width = req.step_width;
    int num_step = req.step_count;
    dt_ = req.dt;
    float theta = req.theta;
    double swing_height = req.ankle_height;
    // double init_COM_height = thigh_ + shank_;  // SURENA IV initial height 
    
    // DCMPlanner* trajectoryPlanner = new DCMPlanner(COM_height_, t_s, t_ds, dt_, num_step + 2, alpha, theta);
    // Ankle* anklePlanner = new Ankle(t_s, t_ds, swing_height, alpha, num_step, dt_, theta);
    // Vector3d* dcm_rf = new Vector3d[num_step + 2];  // DCM rF
    // Vector3d* ankle_rf = new Vector3d[num_step + 2]; // Ankle rF
    // xiDesired_ = trajectoryPlanner->getXiTrajectory();
    // zmpd_ = trajectoryPlanner->getZMP();
    // xiDot_ = trajectoryPlanner->getXiDot();
    // delete[] dcm_rf;
    // anklePlanner->updateFoot(ankle_rf, -sign);
    // anklePlanner->generateTrajectory();
    // delete[] ankle_rf;
    // onlineWalk_.setDt(req.dt);
    // onlineWalk_.setInitCoM(Vector3d(0.0,0.0,COM_height_));

    // CoMDot_ = trajectoryPlanner->get_CoMDot();
    // //ROS_INFO("trajectory generated");
    // res.result = true;
    // trajSizes_.push_back(dataSize_);
    // trajContFlags_.push_back(false);
    // isTrajAvailable_ = true;
    FootStepPlanner step_planner(torso_);
    ZMPPlanner zmp_planner;
    zmp_planner.setDt(dt_);
    if(use_file){
        string config_path = ros::package::getPath("trajectory_planner") + "/config/config.yaml";
        zmp_planner.setConfigPath(config_path);
    }else{
        step_planner.setParams(step_len, step_width, num_step, 0.0, theta);
        step_planner.planSteps();
        zmp_planner.setFootStepsData(step_planner.getFootPrints(), step_planner.getFootYaws());
        zmp_planner.setParams(init_ds, t_ds, t_s, final_ds, swing_height);
    }
    zmp_planner.planInitialDSPZMP();
    zmp_planner.planStepsZMP();
    zmp_planner.planFinalDSPZMP();
    PreviewTraj traj(&zmp_planner, COM_height_, 1.8 / dt_, dt_);
    traj.computeWeight();
    traj.computeTraj();
    vector<Vector3d> com = traj.getCoMPos();
    CoMPos_.insert(CoMPos_.end(), com.begin(), com.end());
    traj.planYawTraj();
    vector<Matrix3d> com_rot = traj.getCoMRot();
    CoMRot_.insert(CoMRot_.end(), com_rot.begin(), com_rot.end());
    
    AnkleTraj ank_traj;
    ank_traj.planInitialDSP();
    ank_traj.planSteps();
    ank_traj.planFinalDSP();
    vector<Vector3d> lank = ank_traj.getLAnklePos();
    lAnklePos_.insert(lAnklePos_.end(), lank.begin(), lank.end());
    vector<Matrix3d> lank_rot = ank_traj.getLAnkleRot();
    lAnkleRot_.insert(lAnkleRot_.end(), lank_rot.begin(), lank_rot.end());
    vector<Vector3d> rank = ank_traj.getRAnklePos();
    rAnklePos_.insert(rAnklePos_.end(), rank.begin(), rank.end());
    vector<Matrix3d> rank_rot = ank_traj.getRAnkleRot();
    rAnkleRot_.insert(rAnkleRot_.end(), rank_rot.begin(), rank_rot.end());

    int trajectory_size = com.size();
    dataSize_ += trajectory_size;
    trajSizes_.push_back(dataSize_);
    trajContFlags_.push_back(false);
    isTrajAvailable_ = true;

    res.result = true;
    res.traj_size = dataSize_;

    return true;
}

bool Robot::generalTrajCallback(trajectory_planner::GeneralTraj::Request  &req,
                                trajectory_planner::GeneralTraj::Response &res)
{
    dt_ = req.dt;
    GeneralMotion* motion_planner = new GeneralMotion(dt_);
    motion_planner->changeInPlace(Vector3d(req.init_com_pos[0], req.init_com_pos[1], req.init_com_pos[2]), 
                                  Vector3d(req.final_com_pos[0], req.final_com_pos[1], req.final_com_pos[2]), 
                                  Vector3d(req.init_com_orient[0], req.init_com_orient[1], req.init_com_orient[2]), 
                                  Vector3d(req.final_com_orient[0], req.final_com_orient[1], req.final_com_orient[2]),
                                  Vector3d(req.init_lankle_pos[0], req.init_lankle_pos[1], req.init_lankle_pos[2]), 
                                  Vector3d(req.final_lankle_pos[0], req.final_lankle_pos[1], req.final_lankle_pos[2]),
                                  Vector3d(req.init_lankle_orient[0], req.init_lankle_orient[1], req.init_lankle_orient[2]), 
                                  Vector3d(req.final_lankle_orient[0], req.final_lankle_orient[1], req.final_lankle_orient[2]),
                                  Vector3d(req.init_rankle_pos[0], req.init_rankle_pos[1], req.init_rankle_pos[2]), 
                                  Vector3d(req.final_rankle_pos[0], req.final_rankle_pos[1], req.final_rankle_pos[2]),
                                  Vector3d(req.init_rankle_orient[0], req.init_rankle_orient[1], req.init_rankle_orient[2]), 
                                  Vector3d(req.final_rankle_orient[0], req.final_rankle_orient[1], req.final_rankle_orient[2]),
                                  req.time);

    int trajectory_size = motion_planner->getLength();

    vector<Vector3d> com_pos = motion_planner->getCoMPos();
    CoMPos_.insert(CoMPos_.end(), com_pos.begin(), com_pos.end());
    vector<Matrix3d> com_rot = motion_planner->getCoMOrient();
    CoMRot_.insert(CoMRot_.end(), com_rot.begin(), com_rot.end());

    vector<Vector3d> lank = motion_planner->getLAnklePos();
    lAnklePos_.insert(lAnklePos_.end(), lank.begin(), lank.end());
    vector<Matrix3d> lank_rot = motion_planner->getLAnkleOrient();
    lAnkleRot_.insert(lAnkleRot_.end(), lank_rot.begin(), lank_rot.end());

    vector<Vector3d> rank = motion_planner->getRAnklePos();
    rAnklePos_.insert(rAnklePos_.end(), rank.begin(), rank.end());
    vector<Matrix3d> rank_rot = motion_planner->getRAnkleOrient();
    rAnkleRot_.insert(rAnkleRot_.end(), rank_rot.begin(), rank_rot.end());

    vector<int> robot_state = motion_planner->getRobotState();
    robotState_.insert(robotState_.end(), robot_state.begin(), robot_state.end());  

    dataSize_ += trajectory_size;
    res.traj_size = dataSize_;
    trajSizes_.push_back(dataSize_);
    trajContFlags_.push_back(false);
    isTrajAvailable_ = true;

    return true;
}

bool Robot::jntAngsCallback(trajectory_planner::JntAngs::Request  &req,
                            trajectory_planner::JntAngs::Response &res)
{
    /*
        ROS service for returning joint angles. before calling this service, 
        you must first call traj_gen service. 
    */
    isTrajAvailable_ = true;
    if (isTrajAvailable_)
    {
        index_ = req.iter;
        double jnt_angs[12];
        Vector3d right_torque(req.right_ft[1], req.right_ft[2], 0.0);
        Vector3d left_torque(req.left_ft[1], req.left_ft[2], 0.0);
        double config[13];
        double jnt_vel[13];
        config[0] = 0;     //Pelvis joint angle
        jnt_vel[0] = 0;  //Pelvis joint velocity
        for(int i = 1; i < 13; i++){
            config[i] = req.config[i-1];
            jnt_vel[i] = req.jnt_vel[i-1];  
        }
        this->spinOnline(req.iter, config, jnt_vel, right_torque, left_torque, req.right_ft[0], req.left_ft[0],
                         Vector3d(req.gyro[0], req.gyro[1], req.gyro[2]),
                         Vector3d(req.accelerometer[0],req.accelerometer[1],req.accelerometer[2]), jnt_angs);
        for(int i = 0; i < 12; i++)
            res.jnt_angs[i] = jnt_angs[i];
        //ROS_INFO("joint angles requested");
    }else{
        ROS_INFO("First call traj_gen service");
        return false;
    }
    
    if (req.iter == dataSize_ - 1 ){ 
        write2File(FKCoM_, dataSize_,"CoM Real");
        write2File(realZMP_, dataSize_, "ZMP Real");
        write2File(realXi_, dataSize_, "Xi Real");
        write2File(FKCoMDot_, dataSize_, "CoM Velocity Real");
        write2File(rSoles_, dataSize_, "Right Sole");
        write2File(lSoles_, dataSize_, "Left Sole");
    }
    //ROS_INFO("joint angles returned");
    return true;
}

bool Robot::resetTrajCallback(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res)
{
    delete[] FKCoM_;
    delete[] FKCoMDot_;
    delete[] realXi_;
    delete[] realZMP_;
    delete[] rSoles_;
    delete[] lSoles_;

    trajSizes_.clear();
    trajContFlags_.clear();
    dataSize_ = 0;
    rSole_ << 0.0, -torso_, 0.0;
    lSole_ << 0.0, torso_, 0.0; 
    isTrajAvailable_ = false;
    Vector3d position(0.0, 0.0, 0.0);
    links_[0]->initPose(position, Matrix3d::Identity(3, 3));
    return true;
}

void Robot::baseOdomPublisher(Vector3d base_pos, Vector3d base_vel, Quaterniond base_quat){

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = base_pos(0);
    odom_trans.transform.translation.y = base_pos(1);
    odom_trans.transform.translation.z = base_pos(2);

    odom_trans.transform.rotation.x = base_quat.x();
    odom_trans.transform.rotation.y = base_quat.y();
    odom_trans.transform.rotation.z = base_quat.z();
    odom_trans.transform.rotation.w = base_quat.w();
    
    baseOdomBroadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = base_pos(0);
    odom.pose.pose.position.y = base_pos(1);
    odom.pose.pose.position.z = base_pos(2);

    odom.pose.pose.orientation.x = base_quat.x();
    odom.pose.pose.orientation.y = base_quat.y();
    odom.pose.pose.orientation.z = base_quat.z();
    odom.pose.pose.orientation.w = base_quat.w();

    odom.twist.twist.linear.x = base_vel(0);
    odom.twist.twist.linear.y = base_vel(1);
    odom.twist.twist.angular.z = base_vel(2);

    //publish the message
    baseOdomPub_.publish(odom);
}

Robot::~Robot(){
    //delete[] links_;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    Matrix3d kp, ki, kcom, kzmp;
    kp << 1,0,0,0,1,0,0,0,0;
    ki = MatrixXd::Zero(3, 3);
    kcom = MatrixXd::Zero(3, 3);
    kzmp = MatrixXd::Zero(3, 3);
    Controller default_ctrl(kp, ki, kzmp, kcom);
    Robot surena(&nh, default_ctrl);
    ros::spin();
}