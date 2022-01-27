#include "Controller.h"


Controller::Controller(Matrix3d K_p, Matrix3d K_i, Matrix3d K_zmp, Matrix3d K_com){
    setK_p_(K_p);
    setK_i_(K_i);
    setK_zmp_(K_zmp);
    setK_com_(K_com);
    //this->K_p_ = K_p;
    //this->K_i_ = K_i;
    //this->K_zmp_ = K_zmp;
    //this->K_com_ = K_com;
    xiErrorInt << 0.0, 0.0, 0.0;
    double deltaZ_ = 0.0;
}

Vector3d Controller::dcmController(Vector3d xiRef, Vector3d xiDotRef, Vector3d xiReal, double deltaZVRP){
    Vector3d xiError = xiRef - xiReal;
    xiErrorInt += xiError * dt_;
    
    Vector3d rRefZMP;
    rRefZMP = xiRef - xiDotRef/sqrt(9.81/deltaZVRP) - (K_p_) * xiError + (K_i_) * xiErrorInt;
    return rRefZMP;
}

Vector3d Controller::comController(Vector3d xCOMRef, Vector3d xDotCOMRef, Vector3d xCOMReal, Vector3d rZMPRef, Vector3d rZMPReal){
    Vector3d xDotStar;
    xDotStar = xDotCOMRef - K_zmp_*(rZMPRef - rZMPReal) + K_com_*(xCOMRef - xCOMReal);
    CoM_ += xDotStar * dt_;
    return CoM_;
}

double Controller::footLenController(double delta_fz_d, double delta_fz, double kp, double kr){
    double deltaZ_dot = kp * (delta_fz_d - delta_fz) - kr*deltaZ_;
    deltaZ_ += deltaZ_dot * dt_;
    return deltaZ_;
}

vector<double> Controller::ankleOrientController(double *bump_sensor_readings, bool left_swing, double k_roll, double k_pitch){
    vector<double> modification_angles;
    double roll_modification;
    double pitch_modification;
    if(left_swing){
        roll_modification = k_roll*((bump_sensor_readings[1] + bump_sensor_readings[3])/2 - (bump_sensor_readings[0] + bump_sensor_readings[2])/2);
        modification_angles.push_back(roll_modification);
        pitch_modification = k_pitch*((bump_sensor_readings[0] + bump_sensor_readings[1])/2 - (bump_sensor_readings[2] + bump_sensor_readings[3])/2);
        modification_angles.push_back(pitch_modification);
    }else{
        roll_modification = k_roll*((bump_sensor_readings[1] + bump_sensor_readings[3])/2 - (bump_sensor_readings[0] + bump_sensor_readings[2])/2);
        modification_angles.push_back(roll_modification);
        pitch_modification = k_pitch*((bump_sensor_readings[0] + bump_sensor_readings[1])/2 - (bump_sensor_readings[2] + bump_sensor_readings[3])/2);
        modification_angles.push_back(pitch_modification);
    }
    return modification_angles;
}

/*vector<double> Controller::ankleOrientController(double *bump_sensor_readings, bool left_swing, double k_roll, double k_pitch){
    vector<int> active_sensors = activeSensors(bump_sensor_readings, left_swing);
    vector<double> modification;
    double roll_modification;
    double pitch_modification;
    int active_size = active_sensors.size();
    if(left_swing){
        switch (active_size)
        {
        case 1:
            roll_modification = k_roll*((bump_sensor_readings[1] + bump_sensor_readings[3])/2 - (bump_sensor_readings[0] + bump_sensor_readings[2])/2);
            pitch_modification = k_pitch*((bump_sensor_readings[0] + bump_sensor_readings[1])/2 - (bump_sensor_readings[2] + bump_sensor_readings[3])/2);
            break;
        case 2:
            if((active_sensors[0] == 0 && active_sensors[1] == 3) || (active_sensors[0] == 1 && active_sensors[1] == 2)){
                pitch_modification = k_pitch*((bump_sensor_readings[0] + bump_sensor_readings[1])/2 - (bump_sensor_readings[2] + bump_sensor_readings[3])/2);
            }else{
                roll_modification = k_roll*((bump_sensor_readings[1] + bump_sensor_readings[3])/2 - (bump_sensor_readings[0] + bump_sensor_readings[2])/2);
                pitch_modification = k_pitch*((bump_sensor_readings[0] + bump_sensor_readings[1])/2 - (bump_sensor_readings[2] + bump_sensor_readings[3])/2);
            }
            break;
        case 3:
            if(active_sensors[0] == 0 && active_sensors[1] == 1 && active_sensors[2] == 2){
                pitch_modification = k_pitch * (abs(bump_sensor_readings[0]-bump_sensor_readings[2]));
                roll_modification = k_roll * (abs(bump_sensor_readings[0] - bump_sensor_readings[1]));
            }else if(active_sensors[0] == 1 && active_sensors[1] == 2 && active_sensors[2] == 3){
                pitch_modification = k_pitch * (abs(bump_sensor_readings[1]-bump_sensor_readings[3]));
                roll_modification = k_roll * (abs(bump_sensor_readings[2] - bump_sensor_readings[3]));
            }else if(active_sensors[0] == 0 && active_sensors[1] == 2 && active_sensors[2] == 3){
                pitch_modification = k_pitch * (abs(bump_sensor_readings[0]-bump_sensor_readings[2]));
                roll_modification = k_roll * (abs(bump_sensor_readings[2] - bump_sensor_readings[3]));
            }else{
                pitch_modification = k_pitch * (abs(bump_sensor_readings[1]-bump_sensor_readings[3]));
                roll_modification = k_roll * (abs(bump_sensor_readings[0] - bump_sensor_readings[1]));
            }
            break;        
        
        default:
            roll_modification = k_roll*((bump_sensor_readings[1] + bump_sensor_readings[3])/2 - (bump_sensor_readings[0] + bump_sensor_readings[2])/2);
            pitch_modification = k_pitch*((bump_sensor_readings[0] + bump_sensor_readings[1])/2 - (bump_sensor_readings[2] + bump_sensor_readings[3])/2);
            break;
        }
    }    
}*/

/*vector<int> Controller::activeSensors(double* bump_sensor_readings, bool left_swing){
    vector<int> active_sensors;
    if(left_swing){
        for(int i=0; i<4; i++){
            if (bump_sensor_readings[i] < 2){
                active_sensors.push_back(i);
            }
        }
    }else{
        for(int i=4; i<8; i++){
            if (bump_sensor_readings[i] < 2){
                active_sensors.push_back(i);
            }
        }
    }
    return active_sensors;
}*/

/*bool Controller::isRightActive(int index){
    bool result;
    if(index%2 == 1){
        result = true;
    }else{
        result = false;
    }
    return result;
}

bool Controller::isUpActive(int index){
    bool result;
    if(index == 0 || index == 1 || index == 4 || index == 5){
        result = true;
    }else{
        result = false;
    }
    return result;
}

int Controller::leastMemberIndex(double *arr, int size){
    int index = 0;
    for(int i=1; i<size; i++){
        if (arr[i] < arr[index]){
            index = i;
        }
    }
    return index;
}*/

void Controller::setDt(double dt){
    this->dt_ = dt;
}

void Controller::setInitCoM(Vector3d init_com){
    this->CoM_ = init_com;
}

void Controller::setK_p_(Matrix3d K_p){
    this->K_p_ = K_p;
}
void Controller::setK_i_(Matrix3d K_i){
    this->K_i_ = K_i;
}
void Controller::setK_zmp_(Matrix3d K_zmp){
    this->K_zmp_ = K_zmp;
}
void Controller::setK_com_(Matrix3d K_com){
    this->K_com_ = K_com;
}

/*int main(){
    Matrix3d kp_;
    kp_<<1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    Matrix3d ki_;
    ki_<<1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    Matrix3d kzmp_;
    kzmp_<<1.0, 1.0, 1.0,
           1.0, 1.0, 1.0,
           1.0, 1.0, 1.0;
    Matrix3d kcom_;
    kcom_<<1.0, 1.0, 1.0,
           1.0, 1.0, 1.0,
           1.0, 1.0, 1.0;
    Vector3d xiRef;
    xiRef<<1.0, 1.0, 1.0;
    Vector3d xiDotRef;
    xiDotRef<<1.0, 1.0, 1.0;
    Vector3d xiReal;
    xiReal<<1.0, 1.0, 1.0;
    double deltaZVRP = 1.0;
    Vector3d result ;






    Controller kosammat(kp_, ki_ , kzmp_, kcom_);
    result = kosammat.dcmController(xiRef, xiDotRef, xiReal, deltaZVRP);
    cout << result <<endl;


}*/



