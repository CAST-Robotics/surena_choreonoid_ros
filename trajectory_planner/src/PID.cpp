#include "PID.h"

PID::PID(Matrix3d kp, Matrix3d ki, Matrix3d kd, double timeStep){
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
    
    this->intI_ = 0.0;
    this->prevoiusError_ = 0.0;
    this->dt_ = timeStep;
    this->xi_error_ = xi_error;
    xi_error[3] = {0.0, 0.0, 0.0};
    this->nh = nh;
    ros::init(argc, argv, "dcm_controller");
    ros::ServiceServer service = nh.advertise("dcmcontroller", dcmController)
}

//double PID::getOutput(double desiredValue, double currentValue){
//    /*
//        computes next plant input
//    */
//    double error = currentValue - desiredValue;
//    this->intI_ += error * dt_;
//    double deriv = (error - this->prevoiusError_) / dt_;
//    this->prevoiusError_ = error;
//
//    return kp_ * error + ki_ * this->intI_ + kd_ * deriv;
//}

bool PID::dcmController(trajectory_planner::DCMController::Request &req,
                        trajectory_planner::DCMController::Response &res){
                            double r_ref[3];
                            Matrix3d Kp = kp*kp;
                            Matrix3d Ki = ki*ki;
                            for(int i=0; i<3; i++){
                                xi_error[i] += req.xi_real[i] - req.xi_ref[i];
                            }
                            for(int i=0; i<3; i++){
                                res.r_zmp_ref[i] = req.xi_ref[i] - ((req.xi_dot_ref[i])/sqrt(9.81/req.deltaZVRP)) + 
                                (Kp(i,0)*(req.xi_real[0]-req.xi_ref[0]) + Kp(i,1)*(req.xi_real[1]-req.xi_ref[1]) + 
                                Kp(i,2)*(req.xi_real[2]-req.xi_ref[2])) + (Ki(i,0)*xi_error[0] + Ki(i,1)*xi_error[1] + Ki(i,2)*xi_error[2]);    
                            }
                            return true
                        }

