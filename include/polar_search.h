#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <se3.hpp>

namespace dm
{
    class BlockMatch;

    class PolarSearch
    {
        friend class BlockMatch;

    public:
        PolarSearch() = default;

        void operator()(
                const Eigen::Vector2d &refPoint,
                const Sophus::SE3d &refPose, const Sophus::SE3d &currPose
        );

        Eigen::Vector2d &getPolarCoef()
        {
            return polarCoef;
        }

        const Eigen::Vector2d &getPolarCoef() const
        {
            return polarCoef;
        }

    private:
        Eigen::Vector2d polarCoef;
        Eigen::Vector2d pointStart;
        Eigen::Vector2d pointEnd;
    };

}

