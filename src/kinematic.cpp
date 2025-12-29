#include "kinematic.h"

/**
    * @param mykinematic.x 位置x
    * @param mykinematic.y 位置y
    * @param delta_f 前轮转向控制量
    * @param accel 加速度
    * @param ref_delta 名义控制输入
    * @param ref_yaw 名义偏航角
*/


// KinematicModel::KinematicModel(double x, double y, double psi, double v, double L, double dt):mykinematic{x, y, psi, v, L, dt}{};

// KinematicModel::~KinematicModel(){}

void KinematicModel::updatestate(double accel, double delta_f){
    x = x + v * cos(yaw)*dt;
    y = y + v * sin(yaw)*dt;
    yaw = yaw + ((v * tan(delta_f)) / L) * dt;
    // v = v + accel * dt;
    // std::cout << "x = " << x << " y = " << y << " yaw = " << yaw << " v = " << v << std::endl;
}

std::tuple<double, double, double, double> KinematicModel::getstate(){
    // return {mykinematic.x, mykinematic.y, mykinematic.psi, mykinematic.v};
    return {x, y, yaw, v};
}

// std::vector<Eigen::MatrixXd> KinematicModel::statespace(double ref_delta, double ref_yaw){
//     Eigen::MatrixXd A(3, 3), B(3, 2);
    
//     A << 1.0, 0.0, -mykinematic.v*sin(ref_yaw)*mykinematic.dt,
//          0.0, 1.0, mykinematic.v*cos(ref_yaw)*mykinematic.dt,
//          0.0, 0.0, 1.0;

//     B << cos(ref_yaw)*mykinematic.dt, 0.0,
//          sin(ref_yaw)*mykinematic.dt, 0.0,
//          (tan(ref_delta)/mykinematic.L)*mykinematic.dt, mykinematic.v/(mykinematic.L*cos(ref_delta)*cos(ref_delta));

//     return {A, B};
// }