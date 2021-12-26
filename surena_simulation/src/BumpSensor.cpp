#include <BumpSensor.h>

BumpSensor::BumpSensor(ros::NodeHandle* nh, double max_h){
    this->nh_ = nh;
    this->maxValue_ = max_h;
    bumpServer_ = nh->advertiseService("/traj_gen", &BumpSensor::sensorCallback, this);
    // Define Obstacles in the simulation environmenrt
    ifstream obs_pos;
    obs_pos.open("../config/obstacles.txt");
    string line, type;
    while(! obs_pos.eof()){
        getline(obs_pos, line);
        if(line == "-"){
            getline(obs_pos, line);
            vector<double> values, parameters;
            this->readOptions(line, values);
            getline(obs_pos, type);
            this->readOptions(line, parameters);
            Obstacle o(values[0], values[1], values[2], values[3], type, parameters);
            this->obstacles_.push_back(o);
        }
    }
    obs_pos.close();
    // Define Robot Sole Dimentions
    ifstream sole_pos;
    sole_pos.open("../config/sole.txt");
    sensorPos_ = new Matrix4d[8];
    for(int i = 0; i < 8; i ++){
        sensorPos_[i] = Matrix4d::Identity(4,4);
        getline(sole_pos, line);
        vector<double> values;
        sensorPos_[i](0,3) = values[0];
        sensorPos_[i](1,3) = values[1];
        sensorPos_[i](2,3) = values[2];
    }
    sole_pos.close();
}

bool BumpSensor::sensorCallback(surena_simulation::bump::Request  &req,surena_simulation::bump::Response &res){
    res.bump_vals = {maxValue_, maxValue_, maxValue_, maxValue_, maxValue_, maxValue_, maxValue_, maxValue_};
    Matrix4d right_ankle, left_ankle;
    right_ankle << req.right_trans[0], req.right_trans[1], req.right_trans[2], req.right_trans[3],
                   req.right_trans[4], req.right_trans[5], req.right_trans[6], req.right_trans[7],
                   req.right_trans[8], req.right_trans[9], req.right_trans[10], req.right_trans[11],
                   req.right_trans[12], req.right_trans[13], req.right_trans[14], req.right_trans[15];
    left_ankle << req.left_trans[0], req.left_trans[1], req.left_trans[2], req.left_trans[3],
                   req.left_trans[4], req.left_trans[5], req.left_trans[6], req.left_trans[7],
                   req.left_trans[8], req.left_trans[9], req.left_trans[10], req.left_trans[11],
                   req.left_trans[12], req.left_trans[13], req.left_trans[14], req.left_trans[15];
    for(int i = 0; i < 4; i++){
        double height = 0;
        double x, y, z;
        x = (left_ankle * sensorPos_[i])(0,3);
        y = (left_ankle * sensorPos_[i])(1,3);
        z = (left_ankle * sensorPos_[i])(2,3);
        for(int j = 0; j < obstacles_.size(); j++){
            if (obstacles_[j].profile(x, y) > height){
                height = obstacles_[j].profile(x, y);
            }
        }
        if(z - height < maxValue_)
            res.bump_vals[i] = z - height;
        else
            res.bump_vals[i] = maxValue_;
    }
    return true;
}

void BumpSensor::readOptions(string opt, vector<double> (&out)){
    stringstream ss(opt);
    float i;

    while (ss >> i){
        out.push_back(i);

        if (ss.peek() == ',')
        ss.ignore();
    }
}

BumpSensor::~BumpSensor(){
    delete sensorPos_;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "bump_sensor");
    ros::NodeHandle nh;
    BumpSensor sensor(&nh, 0.02);
}