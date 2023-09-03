#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace dm
{

    class BlockMatch
    {
    public:
        BlockMatch() = delete;

        explicit BlockMatch(int windowSize) : windowSize(windowSize)
        {

        }

        void operator()(
                const cv::Mat &refImg, const cv::Mat &currImg,
                const Eigen::Vector2d &refPixelPoint,
                const Eigen::Vector2d &currPixelPoint
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
        int windowSize;
    };

}

