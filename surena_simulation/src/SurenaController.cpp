#include <cnoid/SimpleController>
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

public:

    void setParams(){

    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ros::ServiceClient client=nh.serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        trajectory_planner::Trajectory traj;
        traj.request.alpha = 0.5;
        traj.request.t_double_support = 0.2;
        traj.request.t_step = 0.8;
        traj.request.step_length = 0.7;
        traj.request.COM_height = 0.69;
        traj.request.step_count = 6;
        traj.request.ankle_height = 0.02;
        result = client.call(traj);
        ioBody = io->body();
        dt = io->timeStep();


        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;

        return true;
    }
    virtual bool control() override
    {
        ros::ServiceClient client2=nh.serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
        trajectory_planner::JntAngs jntangs;
        jntangs.request.iter = 0;
        double jnts[12];
        if (result){

            jnts = client2.call(jntangs);
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
            return true;
        }
        if (idx<5800){
            idx ++;
        }
    }
};
