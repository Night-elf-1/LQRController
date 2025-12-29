#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

class KinematicModel{
   public:
        double x, y, yaw, v, L, dt;
    public:
        KinematicModel(double x, double y, double psi, double v, double L, double dt) : x(x), y(y), yaw(psi), v(v), L(L), dt(dt){};
        ~KinematicModel(){};

        void updatestate(double accel, double delta_f);

        std::tuple<double, double, double, double> getstate();

};