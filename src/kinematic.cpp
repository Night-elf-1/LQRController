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
