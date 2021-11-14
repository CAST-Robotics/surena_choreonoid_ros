#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/GeneralTraj.h"

using namespace std;
using namespace cnoid;
using namespace Eigen;
/*
const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    8000.0, 8000.0, 8000.0 };
*/
//Surena IV
const double pgain[] = {
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 8000.0,
    8000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0, 3000.0, 3000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };


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
    virtual bool configure(SimpleControllerConfig* config) override
    {
        //config->sigChanged().connect();
        return true;
    }



    virtual bool initialize(SimpleControllerIO* io) override
    {
        dt = io->timeStep();
        /* DCM Walk
        ros::ServiceClient client=nh.serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        trajectory_planner::Trajectory traj;
        traj.request.alpha = 0.44;
        traj.request.t_double_support = 0.1;
        traj.request.t_step = 1;
        traj.request.step_length = 0.5 / 3.6 * 1;
        traj.request.COM_height = 0.63;
        traj.request.step_count = 4;
        traj.request.ankle_height = 0.025;
        traj.request.dt = dt;
        size_ = int(((traj.request.step_count + 2) * traj.request.t_step + 1) / traj.request.dt);
        client.call(traj);
        result = traj.response.result;
        */

        // General Motion
        ros::ServiceClient client=nh.serviceClient<trajectory_planner::GeneralTraj>("/general_traj");
        trajectory_planner::GeneralTraj general_traj;
        general_traj.request.init_com_pos = {0, 0, 0.71};
        general_traj.request.init_com_orient = {0, 0, 0};
        general_traj.request.final_com_pos = {0, 0, 0.71};
        general_traj.request.final_com_orient = {0, 0, 0};

        general_traj.request.init_lankle_pos = {0, 0.1, 0};
        general_traj.request.init_lankle_orient = {0, 0, 0};
        general_traj.request.final_lankle_pos = {0, 0.1, 0};
        general_traj.request.final_lankle_orient = {0, 0, 0};

        general_traj.request.init_rankle_pos = {0, -0.1, 0};
        general_traj.request.init_rankle_orient = {0, 0, 0};
        general_traj.request.final_rankle_pos = {0, -0.1, 0.02};
        general_traj.request.final_rankle_orient = {0, 0, 0.2};

        general_traj.request.time = 2;
        general_traj.request.dt = dt;
        result = true;

        client.call(general_traj);
        cout << "client called" << endl;
        size_ = general_traj.response.duration;
        cout << "size = " << size_ << endl;
        //

        ioBody = io->body();
        leftForceSensor = ioBody->findDevice<ForceSensor>("LeftAnkleForceSensor");
        rightForceSensor = ioBody->findDevice<ForceSensor>("RightAnkleForceSensor");
        io->enableInput(leftForceSensor);
        io->enableInput(rightForceSensor);
        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro");
        io->enableInput(gyro);

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            //qref.push_back(joint->q());
            qref[i] = joint->q();
            qold[i] = qref[i];
            
        }
       // qold = qref;
        return true;
    }
    virtual bool control() override
    {
        ros::ServiceClient jnt_client = nh.serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
        trajectory_planner::JntAngs jntangs;
        
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
            jntangs.request.config[j] = cur_q[sr1Index_[j]];
            jntangs.request.jnt_vel[j] = (cur_q[sr1Index_[j]] - qold[sr1Index_[j]]) / dt;
            }
        jntangs.request.accelerometer = {accelSensor->dv()(0),accelSensor->dv()(1),accelSensor->dv()(2)};
        jntangs.request.gyro = {float(gyro->w()(0)),float(gyro->w()(1)),float(gyro->w()(2))};

        if (result){

            jnt_client.call(jntangs);

            for (int j=0; j<12; j++)
                qref[sr1Index_[j]] = jntangs.response.jnt_angs[j];
                
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                double q = joint->q();
                double dq = (q - qold[i]) / dt;
                double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
                qold[i] = q;
                joint->u() = u;
            }
        }
        //cout << "idx: " << idx << endl;
        if (idx < size_ - 1){
            idx ++;
        }
        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SurenaController)