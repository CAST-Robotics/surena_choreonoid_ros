#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/JntAngs.h"

using namespace std;
using namespace cnoid;
using namespace Eigen;

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

    int size_;

public:
    virtual bool configure(SimpleControllerConfig* config) override
    {
        //config->sigChanged().connect();
        return true;
    }



    virtual bool initialize(SimpleControllerIO* io) override
    {
        ros::ServiceClient client=nh.serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        trajectory_planner::Trajectory traj;
        traj.request.alpha = 0.5;
        traj.request.t_double_support = 0.3;
        traj.request.t_step = 1.4;
        traj.request.step_length = 0.25;
        traj.request.COM_height = 0.6;
        traj.request.step_count = 6;
        traj.request.ankle_height = 0.05;
        dt = io->timeStep();
        traj.request.dt = dt;
        size_ = int(((traj.request.step_count + 2) * traj.request.t_step + 1) / traj.request.dt);
        client.call(traj);
        result = traj.response.result;
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

        for (int j=0; j<6; j++){
                if (j == 0) {
                    jntangs.request.config[j] = qold[j + 2];
                    jntangs.request.config[6+j] = qold[j + 13 + 2];
                    jntangs.request.jnt_vel[j] = (cur_q[j + 2] - qold[j + 2]) / dt;
                    jntangs.request.jnt_vel[6+j] = (cur_q[j + 13 + 2] - qold[j + 13 + 2]) / dt;
                }else if (j == 1 || j == 2){
                    jntangs.request.config[j] = qold[j - 1];
                    jntangs.request.config[6+j] = qold[j + 13 - 1];
                    jntangs.request.jnt_vel[j] = (cur_q[j - 1] - qold[j - 1]) / dt;
                    jntangs.request.jnt_vel[6+j] = (cur_q[j + 13 - 1] - qold[j + 13 - 1]) / dt;
                    
                }else {
                    jntangs.request.config[j] = qold[j];
                    jntangs.request.config[6+j] = qold[j + 13];
                    jntangs.request.jnt_vel[j] = (cur_q[j] - qold[j]) / dt;
                    jntangs.request.jnt_vel[6+j] = (cur_q[j + 13] - qold[j + 13]) / dt;
                }
            }
        //jntangs.request.config = config;
        jntangs.request.accelerometer = {accelSensor->dv()(0),accelSensor->dv()(1),accelSensor->dv()(2)};
        jntangs.request.gyro = {float(gyro->w()(0)),float(gyro->w()(1)),float(gyro->w()(2))};
        float jnts[12];
        if (result){

            jnt_client.call(jntangs);
            //jnts = jntangs.response.jnt_angs;
            for(int i = 0; i<12; i++){
                jnts[i] = jntangs.response.jnt_angs[i];
            }
            for (int j=0; j<6; j++){
                
                if (j == 0 || j == 1) {
                    qref[j] = jnts[j + 1];
                    qref[13+j] = jnts[j + 6 + 1];
                }else if (j == 2){
                    qref[j] = jnts[j - 2];
                    qref[13+j] = jnts[j + 6 - 2];
                }else {
                    qref[j] = jnts[j];
                    qref[13+j] = jnts[j+6];
                }
            }
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                double q = joint->q();
                double dq = (q - qold[i]) / dt;
                double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
                qold[i] = q;
                joint->u() = u;
            }
        }
        
        if (idx < size_ - 1){
            idx ++;
        }
        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SurenaController)