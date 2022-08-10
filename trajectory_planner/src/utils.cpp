#include "utils.h"

Matrix3d skewSym(const Vector3d &vec){
    Matrix3d skew;
    skew << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return skew;
}

void ExpSO3(const Vector3d &vec, Matrix3d &output){
    Matrix3d A = skewSym(vec);
    double theta = vec.norm();
    if(theta == 0){
        output = Matrix3d::Identity();
    }else{
        output = Matrix3d::Identity() + (sin(theta) / theta) * A + (1 - cos(theta)) / (theta * theta) * A * A;
    }
}

MatrixXd matrixPower(const MatrixXd &A, const int &n){
    MatrixXd output;
    if(n == 0){
        output = MatrixXd::Identity(A.rows(), A.cols());
    }else if(n == 1){
        output = A;
    }else{
        MatrixXd B = A;
        for(int i = 1; i < n; i++){
            B = B * A;
        }
        output = B;
    }
    return output;
}

double factorial(const int &n){
    double result = 1;
    for(int i = 1; i <= n; i++){
        result *= i;
    }
    return result;
}

Matrix3d gamma(const Vector3d &vec, int n) {
    Matrix3d output;
    Matrix3d A = skewSym(vec);
    Matrix3d R;
    ExpSO3(vec, R);
    double theta = vec.norm();
    Matrix3d I = Matrix3d::Identity();
    
    if(theta <= 1e-10){
        output = (1 / factorial(n)) * I;
        return output;
    }

    Matrix3d S = Matrix3d::Identity();
    for(int i = 1; i <= n; i++){
        S = S + matrixPower(A, i) / factorial(i);
    }

    if(n == 0){
        output = R;
        return output;
    }else if(n % 2 != 0){
        output = (1 / factorial(n)) * I + ((pow(-1, (n + 1) / 2)) / pow(theta, n + 1)) * A * (R - S);
        return output;
    }else{
        output = (1 / factorial(n)) * I + (pow(-1, n / 2) / pow(theta, n)) * (R - S);
        return output;
    }
}


MatrixXd adjoint(const MatrixXd &X){
    int n = X.rows();
    int size = (n - 2) * 3;
    MatrixXd output = MatrixXd::Zero(size, size);
    Matrix3d R = X.block(0, 0, 3, 3);
    
    for(int i = 0; i < size; i += 3){
        output.block(i, i, 3, 3) = R;
    }

    output.block(3, 0, 3, 3) = skewSym(X.block(0, 3, 3, 1)) * R;
    output.block(6, 0, 3, 3) = skewSym(X.block(0, 4, 3, 1)) * R;
    output.block(9, 0, 3, 3) = skewSym(X.block(0, 5, 3, 1)) * R;
    output.block(12, 0, 3, 3) = skewSym(X.block(0, 6, 3, 1)) * R;
    return output;
}