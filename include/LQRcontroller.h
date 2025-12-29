#ifndef LQRCONTROLLER_H
#define LQRCONTROLLER_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>

class LQRController {
public:
    // 参数：轴距 L，采样时间 dt
    LQRController(double L, double dt) : L_(L), dt_(dt) {}

    // 计算控制量：返回 [v_cmd, delta_cmd]
    std::pair<double, double> computeControl(const Eigen::Vector3d& current_state, 
                                            const Eigen::Vector3d& ref_state,
                                            double ref_v, double ref_delta);

private:
    double L_;
    double dt_;
    int max_iter = 150;
    double eps = 1e-6;

    Eigen::Matrix3d solveDARE(const Eigen::Matrix3d& A, const Eigen::Matrix<double, 3, 2>& B, 
                             const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R);
};

#endif // !LQRCONTROLLER_H