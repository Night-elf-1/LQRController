#ifndef COMMON_H // 如果没有定义 COMMON_H
#define COMMON_H // 定义 COMMON_H
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <tuple>
#include <limits>

using namespace std;

#define YAW_P2P(angle) fmod(fmod((angle) + M_PI, 2 * M_PI) - 2 * M_PI, 2 * M_PI) + M_PI
static inline bool finish = true;
static inline int N_IND_SEARCH = 10;
static inline int target_ind = 0;
static inline double goal_dis = 5.0;

struct parameters
{
    int L = 3.0;
    int NX = 3, NU = 2, NP = 50, NC = 5;
    double dt = 0.1, row = 10;
};

class CommonFunction
{
public:
public:
    std::vector<double> calculateReferenceSpeeds(const std::vector<double> &curvatures, const double &max_speed);

    std::tuple<int, double> calc_nearest_index(double current_x, double current_y, vector<double> &cx, vector<double> &cy, vector<double> &cyaw);

    void smooth_yaw(std::vector<double> &cyaw);

    std::tuple<int, double> calc_ref_trajectory(double current_x, double current_y, vector<double> &cx, vector<double> &cy, vector<double> &cyaw);

    double normalize_angle(double angle);
};

#endif