#include "Link.h"

_Link::_Link(short int ID, _Link* parent, Vector3d a, Vector3d b, double m, Matrix3d inertia){
    this->ID_ = ID;
    this->parent_ = parent;
    this->a_ = a;
    this->b_ = b;
    this->m_ = m;
    this->I_ = inertia;

    this->q_ = 0.0;
    this->dq_ = 0.0;
    this->ddq_ = 0.0;

 }

 short int _Link::getID(){
    return this->ID_;
 }

double _Link::q(){
    return this->q_;
}

void _Link::update(double q, double dq, double ddq){
    this->q_ = q;
    this->dq_ = dq;
    this->ddq_ = ddq;
}

Vector3d _Link::getPose(){
    return this->p_;
}

Matrix3d _Link::getRot(){
    return this->R_;
}

_Link* _Link::getParent(){
    return this->parent_;
}

Matrix3d _Link::rodrigues(Vector3d w, double dt){
    // Helper Function for calculating attitude in Forward Kinematics

    if (w.norm() < numeric_limits<double>::epsilon()){
        return Matrix3d::Identity(3,3);
    }
    else{
        Vector3d wn = w/w.norm();
        double th = w.norm() * dt;
        Matrix3d w_wedge;
        w_wedge << 0.0, -wn(2), wn(1),
                   wn(2), 0.0, -wn(0),
                   -wn(1), wn(0), 0.0;
        Matrix3d R = Matrix3d::Identity(3,3) + w_wedge * sin(th) + w_wedge * w_wedge * (1 - cos(th));
        return R;
    }
}

MatrixXd _Link::FK(){
    if(this->ID_ == 0){
        return this->transformation();
    }
    else{
        this->parent_->FK();
        this->p_ = this->parent_->getRot() * this->b_ + this->parent_->p_;
        this->R_ = this->parent_->getRot() * this->rodrigues(this->a_, this->q_);
        return this->transformation();
    }
}

MatrixXd _Link::transformation(){
    // returns homogeneous transformation matrix
    MatrixXd T(4,4);
    T << this->R_(0,0), this->R_(0,1), this->R_(0,2), this->p_(0),
         this->R_(1,0), this->R_(1,1), this->R_(1,2), this->p_(1),
         this->R_(2,0), this->R_(2,1), this->R_(2,2), this->p_(2),
         0.0, 0.0, 0.0, 1.0;
    return T;
}

MatrixXd _Link::updateJacobian(){
    vector<_Link> idx;
    idx.push_back(*this);
    _Link base = *(this->getParent());
    while (base.getID() != 0){
        base = *base.getParent();
        idx.push_back(base);
    }
    MatrixXd jacobian =  MatrixXd::Zero(6,idx.size());
    Vector3d target = base.getPose();
    for(int n = idx.size() - 2; n >= 0; n--){
        _Link mom = *(idx[n].getParent());
        Vector3d a = mom.getRot() * idx[n].a_;
        jacobian.block<3,1>(0,n) = a.cross(target - idx[n].getPose());
        jacobian.block<3,1>(3,n) = a;
    }

    return jacobian;
}

_Link::~_Link(){

}
