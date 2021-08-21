#include <cnoid/SimpleController>
#include <cnoid/ForceSensor>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/JntAngs.h"

using namespace std;
using namespace cnoid;

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
        traj.request.t_double_support = 0.2;
        traj.request.t_step = 1.4;
        traj.request.step_length = 0.25;
        traj.request.COM_height = 0.6;
        traj.request.step_count = 6;
        traj.request.ankle_height = 0.05;
        dt = io->timeStep();
        traj.request.dt = dt;
        client.call(traj);
        result = traj.response.result;
        ioBody = io->body();
        leftForceSensor = ioBody->findDevice<ForceSensor>("LeftAnkleForceSensor");
        rightForceSensor = ioBody->findDevice<ForceSensor>("RightAnkleForceSensor");
        io->enableInput(leftForceSensor);
        io->enableInput(rightForceSensor);


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

        jntangs.request.left_ft = {leftForceSensor->f().z(),
                                   leftForceSensor->tau().x(),
                                   leftForceSensor->tau().y()};
        jntangs.request.right_ft = {rightForceSensor->f().z(),
                                   rightForceSensor->tau().x(),
                                   rightForceSensor->tau().y()};
        jntangs.request.iter = idx;
        float jnts[12];
        if (result){

            jnt_client.call(jntangs);
            //jnts = jntangs.response.jnt_angs;
            for(int i = 0; i<12; i++){
                jnts[i] = jntangs.response.jnt_angs[i];
            }
            for (int j=0; j<6; j++){
                qref[j] = jnts[j];
                qref[13+j] = jnts[j+6];
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
        
        if (idx < 11.4 / dt - 1){
            idx ++;
        }
        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SurenaController)