#pragma once

#include <string>

#include <Eigen/Core>

namespace dm
{
    const int BORDER = 20;  // 图像边缘
    const int WIDTH = 640;  // 图像宽度
    const int HEIGHT = 480;  // 图像高度
    const int WINDOW_SIZE = 3;  // 快搜索的窗口半径大小
    const int ITERATIONS = 3;  // 建图循环次数

    // 相机内参
    const double F_X = 481.2;
    const double F_Y = -480.0;
    const double C_X = 319.5;
    const double C_Y = 239.5;

    // 定义深度的收敛范围
    const double DEPTH_MAX = 2.0;
    const double DEPTH_MIN = 1.0;

    const std::string MATCH_METHOD = "NCC"; // 块匹配方法定义 可选["SSD", "SAD", "NCC"]
    const int POLAR_GAP_PIXEL = 2;  // 极线搜索的间距，决定了建图的稠密程度
    const double NCC_MATCH_THRESHOLD = 0.85;  // 使用NCC方法块匹配的阈值
    const double PREDICT_FUNC_SIGMA = 0.3;  // 深度滤波器观测方程中的初始标准差
}
