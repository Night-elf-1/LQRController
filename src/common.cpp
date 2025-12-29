#include "common.h"

using namespace std;

double CommonFunction::normalize_angle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

std::vector<double> CommonFunction::calculateReferenceSpeeds(const std::vector<double> &curvatures, const double &max_speed)
{
    std::vector<double> referenceSpeeds;
    // double max_speed = 2.0; // 最大速度设为2.0 m/s
    for (double k : curvatures)
    {
        // 假设曲率半径与速度成线性关系，曲率越大，速度越低
        double speed = max_speed * (1 - 3 * k);            // 假设最大曲率对应于2π的曲率半径
        speed = std::max(1.0, std::min(max_speed, speed)); // 限制速度在0到max_speed之间
        referenceSpeeds.push_back(speed);
    }
    return referenceSpeeds;
}

void CommonFunction::smooth_yaw(std::vector<double> &cyaw)
{
    for (int i = 0; i < cyaw.size() - 1; i++)
    {
        double dyaw = cyaw[i + 1] - cyaw[i];

        while (dyaw > M_PI / 2.0)
        {
            cyaw[i + 1] -= M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
        while (dyaw < -M_PI / 2.0)
        {
            cyaw[i + 1] += M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
    }
}

std::tuple<int, double> CommonFunction::calc_nearest_index(double current_x, double current_y, vector<double> &cx, vector<double> &cy, vector<double> &cyaw)
{
    double mind = numeric_limits<double>::max(); // 初始化一个变量 mind，用于记录最小的距离平方值
    double ind = 0;                              // 初始化索引 ind，用于存储找到的最近轨迹点的索引

    for (int i = 0; i < cx.size(); i++)
    {
        double idx = current_x - cx[i];
        double idy = current_y - cy[i];
        double d_e = std::sqrt(idx * idx + idy * idy);

        if (d_e < mind)
        {
            mind = d_e;
            ind = i;
        }
    }

    double e_d = -1 * sin(cyaw[ind]) * (current_x - cx[ind]) + cos(cyaw[ind]) * (current_y - cy[ind]);
    // std::cout << "new e_d = " << e_d << std::endl;
    // std::cout << "e_d = " << mind << "min_index = " << ind << std::endl;

    return std::make_tuple(ind, e_d);
}

std::tuple<int, double> CommonFunction::calc_ref_trajectory(double current_x, double current_y, vector<double> &cx, vector<double> &cy, vector<double> &cyaw){
    auto [ind, d_e] = calc_nearest_index(current_x, current_y, cx, cy, cyaw);

    // if (target_ind > ind)
    // {
    //     ind = target_ind;
    // }
    // else
    // {
    //     target_ind = ind;
    // }

    // if (target_ind >= ind) ind = target_ind + 1;
    // if (ind >= cx.size() - 1){
    //     ind = cx.size();
    // }
    // target_ind = ind;

    return std::make_tuple(ind, d_e);
}
