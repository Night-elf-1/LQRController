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

    // Eigen::Matrix3d P_out;
    // Eigen::Matrix3d P = Q;
    // for (int i = 0; i < max_iter; ++i) {
    //     Eigen::Matrix2d S = R + B.transpose() * P * B;
    //     Eigen::Matrix2d S_inv =
    //         S.ldlt().solve(Eigen::Matrix2d::Identity());

    //     Eigen::Matrix3d P_next =
    //         A.transpose() * P * A
    //       - A.transpose() * P * B * S_inv * B.transpose() * P * A
    //       + Q;

    //     // 强制对称
    //     P_next = 0.5 * (P_next + P_next.transpose());

    //     if ((P_next - P).norm() / P.norm() < eps) {
    //         P_out = P_next;
    //         return P_out;
    //     }
    //     P = P_next;
    // }
    // return P;

}

std::pair<double, double> LQRController::computeControl(const Eigen::Vector3d& current_state, 
                                                       const Eigen::Vector3d& ref_state,
                                                       double ref_v, double ref_delta) {
    // double yaw = current_state(2);
    
    // // 1. 计算误差状态 (Error State)
    // Eigen::Vector3d error = current_state - ref_state;
    // // 航向角误差正规化
    // while (error(2) > M_PI) error(2) -= 2 * M_PI;
    // while (error(2) < -M_PI) error(2) += 2 * M_PI;

    // // 2. 运动学模型线性化 A, B 矩阵
    // // X_dot = [0, 0, -v*sin(psi); 0, 0, v*cos(psi); 0, 0, 0]
    // Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    // A(0, 2) = -ref_v * sin(ref_state(2)) * dt_;
    // A(1, 2) = ref_v * cos(ref_state(2)) * dt_;

    // Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    // B(0, 0) = cos(ref_state(2)) * dt_;
    // B(1, 0) = sin(ref_state(2)) * dt_;
    // B(2, 0) = (tan(ref_delta) / L_) * dt_;
    // B(2, 1) = ref_v * dt_ / (L_ * pow(cos(ref_delta), 2));

    // // 3. LQR 权重矩阵 (可根据需要调参)
    // Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    // // Q(0, 0) = 10.0; // x误差权重
    // // Q(1, 1) = 10.0; // y误差权重
    // // Q(2, 2) = 1.0;  // yaw误差权重
    // Q(0,0) = 1.0;    // x误差（弱化）
    // Q(1,1) = 20.0;   // y误差（强化）
    // Q(2,2) = 30.0;   // yaw误差（强化）

    // Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    // // R(0, 0) = 1.0;  // 速度指令权重
    // // R(1, 1) = 0.1;  // 转角指令权重
    // R(0,0) = 0.5;    // 速度
    // R(1,1) = 1.0;    // 转角（提高）

    // // 4. 求解离散黎卡提方程
    // Eigen::Matrix3d P = solveDARE(A, B, Q, R);
    // Eigen::Matrix<double, 2, 3> K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

    // // 5. 计算反馈控制量 u = -K * error
    // Eigen::Vector2d u = -K * error;

    // // 最终输出 = 参考控制量 + 反馈补偿
    // double v_cmd = ref_v + u(0);
    // double delta_cmd = ref_delta + u(1);

    // // 物理约束：假设最大转角为 35 度 (约 0.61 rad)
    // double max_steer = 35.0 * M_PI / 180.0; 
    // delta_cmd = std::max(-max_steer, std::min(max_steer, delta_cmd));

    // return {v_cmd, delta_cmd};

    double curr_yaw = current_state(2);
    double ref_yaw = ref_state(2);
    
    // 1. 计算全局误差
    double dx = current_state(0) - ref_state(0);
    double dy = current_state(1) - ref_state(1);
    double d_yaw = curr_yaw - ref_yaw;
    
    // 航向角标准化 (-PI ~ PI)
    while (d_yaw > M_PI) d_yaw -= 2 * M_PI;
    while (d_yaw < -M_PI) d_yaw += 2 * M_PI;

    // 2. 将误差转换到【车辆/路径坐标系】 (这是最关键的一步！)
    // e_lon: 纵向误差 (沿路径切线方向)
    // e_lat: 横向误差 (垂直路径方向)
    double e_lon = cos(ref_yaw) * dx + sin(ref_yaw) * dy;
    double e_lat = -sin(ref_yaw) * dx + cos(ref_yaw) * dy;
    
    // 构建新的状态向量 x = [e_lon, e_lat, e_yaw]
    Eigen::Vector3d error_state;
    error_state << e_lon, e_lat, d_yaw;

    // 3. 基于 Frenet 坐标系的线性化模型 (离散化 A, B 矩阵)
    // 这种模型下的 Q 矩阵物理意义才是固定的：
    // Q(0,0) -> 纵向位置权重, Q(1,1) -> 横向位置权重
    
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0, 0) = 1.0;
    A(0, 1) = 0.0; 
    A(0, 2) = 0.0; // 简化模型，忽略纵向与其他耦合
    A(1, 1) = 1.0;
    A(1, 2) = ref_v * dt_; // 横向误差随航向误差变化
    A(2, 2) = 1.0;

    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = dt_; // 速度输入影响纵向位置
    B(2, 1) = (ref_v / L_) * dt_; // 转角输入影响航向角 (简化: cos(delta)~1)

    // 4. 配置 Q 和 R
    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Q(0,0) = 0.5;   // 纵向误差权重 (不用太在乎前后位置)
    Q(1,1) = 30.0;  // 横向误差权重 (非常重要！必须大)
    Q(2,2) = 40.0;  // 航向误差权重 (非常重要！必须大)

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    R(0,0) = 1.0;   // 加速度/速度修正惩罚
    R(1,1) = 2.0;   // 转向幅度惩罚

    // 5. 求解 DARE (你的求解器代码没问题)
    Eigen::Matrix3d P = solveDARE(A, B, Q, R);
    Eigen::Matrix<double, 2, 3> K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

    // 6. 计算反馈 u = -K * error_state
    Eigen::Vector2d u = -K * error_state;

    // 7. 输出叠加 (前馈 + 反馈)
    double v_cmd = ref_v + u(0);
    double delta_cmd = ref_delta + u(1);
    
    // 8. *** 必须加限幅 ***
    double max_steer = 0.6; // 约 35度
    if (delta_cmd > max_steer) delta_cmd = max_steer;
    if (delta_cmd < -max_steer) delta_cmd = -max_steer;

    return {v_cmd, delta_cmd};

}