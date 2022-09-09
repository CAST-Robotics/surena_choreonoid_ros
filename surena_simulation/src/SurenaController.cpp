#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/GeneralTraj.h"
#include "surena_simulation/bump.h"
#include "std_srvs/Empty.h"

using namespace std;
using namespace cnoid;
using namespace Eigen;
//SR1
/*
const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };
*/
//Surena IV

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    8000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

//SurenaV
/*
const double pgain[] = {
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 8000.0,
    8000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0 };

const double dgain[] = {
    20.0, 20.0, 20.0, 20.0, 20.0, 20.0,
    20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };
*/
class SurenaController : public SimpleController{
  
    bool result;
    ros::NodeHandle nh;
    int idx = 0;
    double dt;
    double qref[29];
    double qold[29];
    BodyPtr ioBody;
    ForceSensor* leftForceSensor;
    ForceSensor* rightForceSensor;
    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;
    int surenaIndex_[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    int sr1Index_[12] = {2, 0, 1, 3, 4, 5, 15, 13, 14, 16, 17, 18};

    int size_;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        dt = io->timeStep();
        // General Motion
        ros::ServiceClient gen_client=nh.serviceClient<trajectory_planner::GeneralTraj>("/general_traj");
        trajectory_planner::GeneralTraj general_traj;
        general_traj.request.init_com_pos = {0, 0, 0.71};
        general_traj.request.init_com_orient = {0, 0, 0};
        general_traj.request.final_com_pos = {0, 0, 0.68};
        general_traj.request.final_com_orient = {0, 0, 0};

        general_traj.request.init_lankle_pos = {0, 0.1, 0};
        general_traj.request.init_lankle_orient = {0, 0, 0};
        general_traj.request.final_lankle_pos = {0, 0.1, 0};
        general_traj.request.final_lankle_orient = {0, 0, 0};

        general_traj.request.init_rankle_pos = {0, -0.1, 0};
        general_traj.request.init_rankle_orient = {0, 0, 0};
        general_traj.request.final_rankle_pos = {0, -0.1, 0};
        general_traj.request.final_rankle_orient = {0, 0, 0};

        general_traj.request.time = 2;
        general_traj.request.dt = dt;

        gen_client.call(general_traj);
        //DCM Walk
        ros::ServiceClient client=nh.serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        trajectory_planner::Trajectory traj;

        traj.request.step_width = 0.0;
        traj.request.alpha = 0.44;
        traj.request.t_double_support = 0.1;
        traj.request.t_step = 1.0;
        traj.request.step_length = -0.17;
        traj.request.COM_height = 0.68;
        traj.request.step_count = 12;
        traj.request.ankle_height = 0.025;
        traj.request.theta = 0.15;
        traj.request.dt = dt;
        result = traj.response.result;
        client.call(traj);

        size_ = int(((traj.request.step_count + 2) * traj.request.t_step + general_traj.request.time) / traj.request.dt);
        // size_ = int((general_traj.request.time) / traj.request.dt);

        result = true;

        
        ioBody = io->body();
        leftForceSensor = ioBody->findDevice<ForceSensor>("LeftAnkleForceSensor");
        rightForceSensor = ioBody->findDevice<ForceSensor>("RightAnkleForceSensor");
        io->enableInput(leftForceSensor);
        io->enableInput(rightForceSensor);
        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro"); //SR1 & SurenaV
        //gyro = ioBody->findDevice<RateGyroSensor>("gyrometer"); //SurenaIV
        io->enableInput(gyro);
        io->enableInput(ioBody->link(0), LINK_POSITION);

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            if(i == 5 || i == 11){
                // Enabling Ankles Position IO (required by bump sensor)
                io->enableInput(joint, LINK_POSITION);
            }
            qref[i] = joint->q();
            qold[i] = qref[i];
            
        }
        return true;
    }
    virtual bool control() override
    {
        ros::ServiceClient bumpSensor = nh.serviceClient<surena_simulation::bump>("/bumpSensor");
        ros::ServiceClient jnt_client = nh.serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
        trajectory_planner::JntAngs jntangs;
        if (idx < size_ - 1){
            jntangs.request.left_ft = {float(leftForceSensor->f().z()),
                                    float(leftForceSensor->tau().x()),
                                    float(leftForceSensor->tau().y())};
            jntangs.request.right_ft = {float(rightForceSensor->f().z()),
                                    float(rightForceSensor->tau().x()),
                                    float(rightForceSensor->tau().y())};
                                    
            jntangs.request.iter = idx;
            double cur_q[ioBody->numJoints()];
            for(int i=0; i < ioBody->numJoints(); ++i){
                    Link* joint = ioBody->joint(i);
                    cur_q[i] = joint->q();
            }

            for (int j=0; j<12; j++){
                jntangs.request.config[j] = cur_q[surenaIndex_[j]];
                jntangs.request.jnt_vel[j] = (cur_q[surenaIndex_[j]] - qold[surenaIndex_[j]]) / dt;
                }
            jntangs.request.accelerometer = {accelSensor->dv()(0),accelSensor->dv()(1),accelSensor->dv()(2)};
            jntangs.request.gyro = {float(gyro->w()(0)),float(gyro->w()(1)),float(gyro->w()(2))};

            // Getting Bump Sensor Values
            surena_simulation::bump bump_msg;
            Matrix4d l_ankle, r_ankle;
            l_ankle.block<3,1>(0, 3) = ioBody->joint(11)->position().translation();
            r_ankle.block<3,1>(0, 3) = ioBody->joint(5)->position().translation();
            l_ankle.block<3,3>(0, 0) = ioBody->joint(11)->position().rotation();
            r_ankle.block<3,3>(0, 0) = ioBody->joint(5)->position().rotation();
            
            Vector3d base_pos = ioBody->link(0)->p();
            Matrix3d base_rot = ioBody->link(0)->R();

            // rotation to quaternion
            Quaterniond base_quat(base_rot);
            
            Vector3d left_ankle_pos = l_ankle.block<3,1>(0, 3);
            Vector3d right_ankle_pos = r_ankle.block<3,1>(0, 3);
            cout << base_pos(0) << "," << base_pos(1) << "," << base_pos(2) << ",";
            cout << base_quat.w() << "," << base_quat.x() << "," << base_quat.y() << "," << base_quat.z() << endl;
            // cout << left_ankle_pos(0) << "," << left_ankle_pos(1) << "," << left_ankle_pos(2) << ",";
            // cout << right_ankle_pos(0) << "," << right_ankle_pos(1) << "," << right_ankle_pos(2) << endl;
            for(int i = 0; i < 16; i ++){
                bump_msg.request.left_trans[i] = l_ankle(i / 4, i % 4);
                bump_msg.request.right_trans[i] = r_ankle(i / 4, i % 4);
            }
            bumpSensor.call(bump_msg);
            jntangs.request.bump = bump_msg.response.bump_vals;
            if (result){

                jnt_client.call(jntangs);

                for (int j=0; j<12; j++)
                    qref[surenaIndex_[j]] = jntangs.response.jnt_angs[j];
                    
                for(int i=0; i < ioBody->numJoints(); ++i){
                    Link* joint = ioBody->joint(i);
                    double q = joint->q();
                    double dq = (q - qold[i]) / dt;
                    double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
                    qold[i] = q;
                    joint->u() = u;
                }
            }
            idx ++;
        }else{
            if(idx == size_ - 1){
                ros::ServiceClient reset_client = nh.serviceClient<std_srvs::Empty>("/reset_traj");
                std_srvs::Empty srv;
                reset_client.call(srv);
            }
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                double q = joint->q();
                double dq = (q - qold[i]) / dt;
                double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
                qold[i] = q;
                joint->u() = u;
            }
            idx ++;
        }
        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SurenaController)