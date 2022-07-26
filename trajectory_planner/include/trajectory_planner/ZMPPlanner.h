#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "yaml-cpp/yaml.h"
#include "MinJerk.h"

using namespace std;
using namespace Eigen;

class ZMPPlanner : private MinJerk {
public:
    ZMPPlanner(double dt, string config_path);
    ~ZMPPlanner();
    void planInitialDSPZMP();
    void planStepsZMP();
    void planFinalDSPZMP();
    Vector3d getZMP(int iter);
    int getTrajSize();
private:
    double dt_;
    YAML::Node config_;
    double initDSPDuration_;
    double DSPDuration_;
    double SSPDuration_;
    double finalDSPDuration_;
    vector<Vector3d> footSteps_;
    int footStepCount_;
    int trajSize_;
    vector<Vector3d> plannedZMP_;

    void parseConfig(YAML::Node config);
};
