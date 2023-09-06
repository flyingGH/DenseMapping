#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace dm
{
    class PolarSearch;

    class BlockMatch
    {
    public:
        using MethodType = double (*)(
                const std::vector<double> &,
                const std::vector<double> &,
                const double &, const double &
        );

        BlockMatch() = delete;

        explicit BlockMatch(const std::string &methodName)
        {
            if (methodName == "NCC")
                method = &NCC;
            else if (methodName == "SAD")
                method = &SAD;
            else if (methodName == "SSD")
                method = &SSD;
            else
                throw std::runtime_error("this is no " + methodName + "in block match!");
        }

        bool operator()(
                const cv::Mat &refImg, const cv::Mat &currImg,
                const Eigen::Vector2d &refPixelPoint,
                PolarSearch &polarSearch
        );

        Eigen::Vector2d &getCurrPixelPoint()
        {
            return currPixelPoint;
        }

        const Eigen::Vector2d &getCurrPixelPoint() const
        {
            return currPixelPoint;
        }

    private:
        Eigen::Vector2d currPixelPoint;

        MethodType method;

        static double SAD(
                const std::vector<double> &refPatch,
                const std::vector<double> &currPatch,
                const double &refMeanPixel,
                const double &currMeanPixel
        );

        static double bilinear(const cv::Mat &img, const Eigen::Vector2d &point);

        static double SSD(
                const std::vector<double> &refPatch,
                const std::vector<double> &currPatch,
                const double &refMeanPixel,
                const double &currMeanPixel
        );

        static double NCC(
                const std::vector<double> &refPatch,
                const std::vector<double> &currPatch,
                const double &refMeanPixel,
                const double &currMeanPixel
        );
    };

}

