#include <Obstacle.h>

Obstacle::Obstacle(double xl, double xh, double yl, double yh, string type, vector<double> params){
    this->type_ = type;
    this->lowerBound_ << xl, yl;
    this->higherBound_ << xh, yh;

    this->params_ = params;
}

Obstacle::~Obstacle(){
    cout << "Obstacle Object Destroyed" << endl;
}

double Obstacle::profile(double x, double y){
    
    if (this->type_ == "cube"){
        return this->cubeProfile(x, y);
    }
    else if (this->type_ == "ramp"){
        return this->rampProfile(x, y);
    }
}

double Obstacle::cubeProfile(double x, double y){
    if(x < higherBound_(0) && x > lowerBound_(0) && y < higherBound_(1) && y > lowerBound_(1)){
        return this->params_[0];
    }
    else{
        return 0;
    }
}

double Obstacle::rampProfile(double x, double y){
    if(x < higherBound_(0) && x > lowerBound_(0) && y < higherBound_(1) && y > lowerBound_(1)){
        return params_[0] * (x - lowerBound_(0)) + params_[1] +
                params_[2] * (y - lowerBound_(1)) + params_[3];
    }
    else{
        return 0;
    }
}