#pragma once

#include "MinJerk.h"

class ZMPPlanner : public MinJerk {
public:
    ZMPPlanner(bool useFile, double dt=0.005);
    ~ZMPPlanner();
    void planInitialDSPZMP();
    void planStepsZMP();
    void planFinalDSPZMP();
    Vector3d getZMP(int iter);
    
private:
    vector<Vector3d> plannedZMP_;
};