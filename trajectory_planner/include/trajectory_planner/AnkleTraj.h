#pragma once

#include "MinJerk.h"

class AnkleTraj : public MinJerk {
public:
    AnkleTraj(bool useFile, double dt=0.005);
    ~AnkleTraj(){}

    void planStance(Vector3d r_foot, Vector3d l_foot, double time);
    void planSteps();
    inline vector<Vector3d> getRAnkle(){return plannedRAnkle_;}
    inline vector<Vector3d> getLAnkle(){return plannedLAnkle_;}

    void planInitialDSP();
    void planFinalDSP();
    
private:
    vector<Vector3d> plannedRAnkle_;
    vector<Vector3d> plannedLAnkle_;
};