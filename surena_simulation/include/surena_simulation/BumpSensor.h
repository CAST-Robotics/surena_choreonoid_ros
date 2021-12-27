#include <ros/ros.h>
#include "surena_simulation/bump.h"

#include "Obstacle.h"

#include "eigen3/Eigen/Eigen"
#include "math.h"
#include "iostream"
#include "vector"
#include "fstream"
#include "string"

using namespace Eigen;
using namespace std;

class BumpSensor{
    public:
        BumpSensor(ros::NodeHandle* nh, double max_h);
        ~BumpSensor();
    private:
        ros::NodeHandle* nh_;
        ros::ServiceServer bumpServer_;
        bool sensorCallback(surena_simulation::bump::Request  &req,
                            surena_simulation::bump::Response &res);

        void sensorCallback();
        void readOptions(string opt, vector<double> (&out));

        Matrix4d* sensorPos_;
        vector<Obstacle> obstacles_;
        double maxValue_;
};