#include <chrono>
#include <cmath>
#include "matplotlibcpp.h"
#include "cubic_spline.hpp"
#include "kinematic.h"
#include "common.h"
#include "LQRcontroller.h"

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char const *argv[])
{
    //============ 生成参考路线 ============
    vector<double> wx({10.0, 60.0, 125.0, 50.0, 60.0, 35.0, -10.0});
    vector<double> wy({0.0, 0.0, 50.0, 65.0, 45.0, 50.0, -20.0});

    Spline2D csp_obj(wx, wy);
    vector<double> r_x;
    vector<double> r_y;
    vector<double> ryaw;       // 航向角
    vector<double> rcurvature; // 曲率
    vector<double> rs;
    for (double i = 0; i < csp_obj.s.back(); i += 1.0)
    { // 计算出路径点X 和 Y
        vector<double> point_ = csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }
    double target_speed = 10.0;
    //============ 生成参考路线 ============
    //============ 类的初始化 ============
    // 初始化 LQR 控制器 (轴距 3.0, dt 0.1)
    LQRController lqr(3.0, 0.1);
    CommonFunction com_function; // 通用函数方法
    parameters param_;
    //============ 类的初始化 ============
    //============ 初始化、计算各种信息 ============
    // vector<double> speed_profile = pids.calculateReferenceSpeeds(rcurvature, target_speed);
    vector<double> speed_profile = com_function.calculateReferenceSpeeds(rcurvature, target_speed);
    Eigen::Vector3d initial_x; // 初始化agv初始状态 x y yaw
    initial_x << 10.0, 5.0, 0.1;
    KinematicModel agv(initial_x(0), initial_x(1), initial_x(2), target_speed, 3.0, 0.1);
    std::vector<double> x_history, y_history;
    com_function.smooth_yaw(ryaw);
    //============ 初始化、计算各种信息 ============
    //============ 循环计算 ============
    plt::figure_size(800, 600); // 设置窗口大小
    while (finish)
    {
        auto [min_index, min_e] = com_function.calc_ref_trajectory(initial_x(0), initial_x(1), r_x, r_y, ryaw);
        double yaw_error = com_function.normalize_angle(ryaw[min_index] - initial_x(2));

        // 准备 LQR 输入
        Eigen::Vector3d current_state = initial_x; // [x, y, yaw]
        Eigen::Vector3d ref_state;
        ref_state << r_x[min_index], r_y[min_index], ryaw[min_index];
        // 计算参考转角 (前馈): delta = atan(L * K)
        double ref_delta = atan2(3.0 * rcurvature[min_index + 5], 1.0);
        double ref_v = speed_profile[min_index];
        // LQR 计算
        auto [v_real, delta_real] = lqr.computeControl(current_state, ref_state, ref_v, ref_delta);

        std::cout << "最近点索引 = " << min_index << "  最近点横向误差 = " << min_e  << " 最近点之间的航向角误差 = " << yaw_error << std::endl;
        std::cout << "输出速度v = " << v_real << " 输出转角delta = " << delta_real << std::endl;

        agv.updatestate(v_real, delta_real);

        auto [temp_x, temp_y, temp_yaw, temp_v] = agv.getstate();

        initial_x << temp_x, temp_y, temp_yaw;
        x_history.push_back(temp_x);
        y_history.push_back(temp_y);

        plt::clf();                            // 清除当前图像
        plt::plot(r_x, r_y, "b--");            // 绘制参考路径
        plt::plot(x_history, y_history, "r-"); // 绘制AGV的实际轨迹
        plt::scatter(std::vector<double>{initial_x(0)}, std::vector<double>{initial_x(1)}, 20.0, {{"color", "green"}});
        plt::xlabel("x[m]");
        plt::ylabel("y[m]");
        plt::axis("equal");
        plt::grid(true);
        plt::pause(0.01); // 暂停以更新图形

        if (min_index >= r_x.size() - 15)
        {
            finish = false;
            std::cout << "航向角误差：" << std::endl;
            std::cout << temp_yaw - ryaw[min_index] << std::endl;
        }
    }

    plt::show(); // 显示最终结果

    return 0;
}
