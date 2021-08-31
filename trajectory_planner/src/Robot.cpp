#include "Robot.h" 

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


Robot::Robot(ros::NodeHandle *nh){

    trajGenServer_ = nh->advertiseService("/traj_gen", 
            &Robot::trajGenCallback, this);
    jntAngsServer_ = nh->advertiseService("/jnt_angs", 
            &Robot::jntAngsCallback, this);
    
    // SURENA IV geometrical params
    
    shank_ = 0.3;     // SR1: 0.3, Surena4: 0.36
    thigh_ = 0.3535;  // SR1: 0.3535, Surena4: 0.37
    torso_ = 0.09;    // SR1: 0.09, Surena4: 0.115

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

    cout << "Robot Object has been Created" << endl;
}

void Robot::spinOnline(int iter, double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer){
    // update joint positions
    for (int i = 0; i < 13; i ++){
        links_[i]->update(config[i], jnt_vel[i], 0.0);
    }
    // Do the Forward Kinematics for Lower Limb
    links_[12]->FK();
    links_[6]->FK();    // update all raw values of sensors and link states
    updateState(config, torque_r, torque_l, f_r, f_l, gyro, accelerometer);    
}

void Robot::updateState(double config[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer){
    
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

    if(leftSwings_ && (!rightSwings_)){
        lSole_ = rSole_ - links_[6]->getPose() + links_[12]->getPose();
        FKCoM_[index_] = lSole_ - links_[12]->getPose();
        FKCoMDot_[index_] = - links_[6]->getVel().block<3,1>(0, 0);
        realXi_[index_] = FKCoM_[index_] + FKCoMDot_[index_] / sqrt(K_G/COM_height_);
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    else if ((!leftSwings_) && rightSwings_){
        Matrix<double, 6, 1> q_dot;
        rSole_ = lSole_ - links_[12]->getPose() + links_[6]->getPose();
        FKCoM_[index_] = rSole_ - links_[6]->getPose();
        FKCoMDot_[index_] = - links_[12]->getVel().block<3,1>(0, 0);
        realXi_[index_] = FKCoM_[index_] + FKCoMDot_[index_] / sqrt(K_G/COM_height_);
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    else{   // double support
        FKCoM_[index_] = rSole_ - links_[6]->getPose();
        FKCoMDot_[index_] = - links_[6]->getVel().block<3,1>(0, 0);
        realXi_[index_] = FKCoM_[index_] + FKCoMDot_[index_] / sqrt(K_G/COM_height_);
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
}

Vector3d Robot::getZMPLocal(Vector3d torque, double fz){
    // Calculate ZMP for each foot
    Vector3d zmp(0.0, 0.0, 0.0);
    if (fz == 0){
        ROS_WARN("No Correct Force Value!");
        return zmp;
    }
    zmp(0) = -torque(1)/fz;
    zmp(1) = -torque(0)/fz;
    return zmp;
}

Vector3d Robot::ZMPGlobal(Vector3d zmp_r, Vector3d zmp_l, double f_r, double f_l){
    // Calculate ZMP during Double Support Phase
    if (f_r + f_l == 0){
        ROS_WARN("No Foot Contact, Check the Robot!");
    }
    //assert(!(f_r + f_l == 0));

    return (zmp_r * f_r + zmp_l * f_l) / (f_r + f_l);
}

void Robot::spinOffline(int iter, double* config){
    
    MatrixXd lfoot(3,1);
    MatrixXd rfoot(3,1);
    Matrix3d attitude = MatrixXd::Identity(3,3);
    MatrixXd pelvis(3,1);
    lfoot << lAnkle_[iter](0), lAnkle_[iter](1), lAnkle_[iter](2);
    rfoot << rAnkle_[iter](0), rAnkle_[iter](1), rAnkle_[iter](2);
    pelvis << comd_[iter](0), comd_[iter](1), comd_[iter](2);
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
    double* choreonoid_only = new double[6] {q[1], q[2], q[0], q[3], q[4], q[5]};
    return choreonoid_only;
}

bool Robot::trajGenCallback(trajectory_planner::Trajectory::Request  &req,
                            trajectory_planner::Trajectory::Response &res)
{
    /*
        ROS service for generating robot COM & ankles trajectories
    */
    ROS_INFO("Generating Trajectory started.");
    size_ = int(((req.step_count + 2) * req.t_step + 1) / req.dt);
    double alpha = req.alpha;
    double t_ds = req.t_double_support;
    double t_s = req.t_step;
    COM_height_ = req.COM_height;
    double step_len = req.step_length;
    int num_step = req.step_count;
    double dt = req.dt;
    double swing_height = req.ankle_height;
    double init_COM_height = thigh_ + shank_;  // SURENA IV initial height 
    
    DCMPlanner* trajectoryPlanner = new DCMPlanner(COM_height_, t_s, t_ds, dt, num_step + 2, alpha);
    Ankle* anklePlanner = new Ankle(t_s, t_ds, swing_height, alpha, num_step, dt);
    Vector3d* dcm_rf = new Vector3d[num_step + 2];  // DCM rF
    Vector3d* ankle_rf = new Vector3d[num_step + 2]; // Ankle rF
    

    for (int i = 0; i < num_step; i++){
        dcm_rf[i+1] << i * step_len, pow(-1, i + 1) * torso_, 0.0;  // pow(-1, i + 1) : for specifing that first swing leg is left leg
        ankle_rf[i+1] << i * step_len, pow(-1, i + 1) * torso_, 0.0;
    }
    dcm_rf[0] << 0.0, 0.0, 0.0;
    dcm_rf[num_step + 1] << dcm_rf[num_step](0), 0.0, 0.0;
    ankle_rf[0] << 0.0, -ankle_rf[1](1), 0.0;
    ankle_rf[num_step + 1] << ankle_rf[num_step](0), -ankle_rf[num_step](1), 0.0;
    cout << "!!!!!!\n";
    trajectoryPlanner->setFoot(dcm_rf);
    trajectoryPlanner->getXiTrajectory();
    Vector3d com(0.0,0.0,init_COM_height);
    comd_ = trajectoryPlanner->getCoM(com);
    zmpd_ = trajectoryPlanner->getZMP();
    delete[] dcm_rf;
    cout << "!!!!!!\n";
    anklePlanner->updateFoot(ankle_rf);
    anklePlanner->generateTrajectory();
    lAnkle_ = anklePlanner->getTrajectoryL();
    rAnkle_ = anklePlanner->getTrajectoryR();
    delete[] ankle_rf;
    ROS_INFO("trajectory generated");
    res.result = true;
    isTrajAvailable_ = true;

    FKCoM_ = new Vector3d[size_];
    FKCoMDot_ = new Vector3d[size_];
    realXi_ = new Vector3d[size_];
    realZMP_ = new Vector3d[size_];
    rSoles_ = new Vector3d[size_];
    lSoles_ = new Vector3d[size_];

    return true;
}

bool Robot::jntAngsCallback(trajectory_planner::JntAngs::Request  &req,
                            trajectory_planner::JntAngs::Response &res)
{
    /*
        ROS service for returning joint angles. before calling this service, 
        you must first call traj_gen service. 
    */
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
                         Vector3d(req.accelerometer[0],req.accelerometer[1],req.accelerometer[2]));
        this->spinOffline(req.iter, jnt_angs);
        for(int i = 0; i < 12; i++)
            res.jnt_angs[i] = jnt_angs[i];
        //ROS_INFO("joint angles requested");
    }else{
        ROS_INFO("First call traj_gen service");
        return false;
    }

    if (req.iter == size_ - 1 ){ 
        write2File(FKCoM_, size_,"CoM Real");
        write2File(realZMP_, size_, "ZMP Real");
        write2File(realXi_, size_, "Xi Real");
        write2File(FKCoMDot_, size_, "CoM Velocity Real");
        write2File(rSoles_, size_, "Right Sole");
        write2File(lSoles_, size_, "Left Sole");
    }
    //ROS_INFO("joint angles returned");
    //cout << req.iter << endl;
    //cout << req.left_ft[0] << "\t" << req.right_ft[0] << endl;
    return true;
}

Robot::~Robot(){
    //delete[] links_;
    delete[] FKCoM_;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;
    Robot surena(&nh);
    ros::spin();
}