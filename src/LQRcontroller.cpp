#include "LQRcontroller.h"

Eigen::Matrix3d LQRController::solveDARE(const Eigen::Matrix3d& A, const Eigen::Matrix<double, 3, 2>& B, 
                                        const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R) {
    Eigen::Matrix3d P = Q;
    for (int i = 0; i < max_iter; ++i) {
        Eigen::Matrix3d P_next = A.transpose() * P * A - 
            A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        if ((P_next - P).norm() < eps) return P_next;
        P = P_next;
    }
    return P;
}

std::pair<double, double> LQRController::computeControl(const Eigen::Vector3d& current_state, 
                                                       const Eigen::Vector3d& ref_state,
                                                       double ref_v, double ref_delta) {
    double yaw = current_state(2);
    
    // 1. 计算误差状态 (Error State)
    Eigen::Vector3d error = current_state - ref_state;
    // 航向角误差正规化
    while (error(2) > M_PI) error(2) -= 2 * M_PI;
    while (error(2) < -M_PI) error(2) += 2 * M_PI;

    // 2. 运动学模型线性化 A, B 矩阵
    // X_dot = [0, 0, -v*sin(psi); 0, 0, v*cos(psi); 0, 0, 0]
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0, 2) = -ref_v * sin(ref_state(2)) * dt_;
    A(1, 2) = ref_v * cos(ref_state(2)) * dt_;

    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = cos(ref_state(2)) * dt_;
    B(1, 0) = sin(ref_state(2)) * dt_;
    B(2, 0) = tan(ref_delta) / L_ * dt_;
    B(2, 1) = ref_v * dt_ / (L_ * pow(cos(ref_delta), 2));

    // 3. LQR 权重矩阵 (可根据需要调参)
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Q(0, 0) = 10.0; // x误差权重
    Q(1, 1) = 10.0; // y误差权重
    Q(2, 2) = 1.0;  // yaw误差权重

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    R(0, 0) = 1.0;  // 速度指令权重
    R(1, 1) = 0.1;  // 转角指令权重

    // 4. 求解离散黎卡提方程
    Eigen::Matrix3d P = solveDARE(A, B, Q, R);
    Eigen::Matrix<double, 2, 3> K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

    // 5. 计算反馈控制量 u = -K * error
    Eigen::Vector2d u = -K * error;

    // 最终输出 = 参考控制量 + 反馈补偿
    double v_cmd = ref_v + u(0);
    double delta_cmd = ref_delta + u(1);

    return {v_cmd, delta_cmd};
}