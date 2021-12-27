#include "eigen3/Eigen/Eigen"
#include "iostream"
#include "math.h"
#include "eigen3/Eigen/Eigen"
#include "string"
#include "vector"

using namespace Eigen;
using namespace std;

class Obstacle{
    public:
        Obstacle(double xl, double xh, double yl, double yh, string type, vector<double> params);
        ~Obstacle();
        double profile(double x, double y);
    private:
        Vector2d lowerBound_;
        Vector2d higherBound_;
        string type_;
        vector<double> params_;

        double cubeProfile(double x, double y);
        double rampProfile(double x, double y);
};