#pragma once

#include <se3.hpp>
#include <Eigen/Dense>

namespace dm
{

    class Triangulation
    {
    public:
        Triangulation() = default;

        void operator()(
                const Sophus::SE3d &refPose, const Sophus::SE3d &currPose,
                const Eigen::Vector2d &refPoint, const Eigen::Vector2d &currPoint
        );

        Eigen::Vector3d &getWorldPoint()
        {
            return worldPoint;
        }

        const Eigen::Vector3d &getWorldPoint() const
        {
            return worldPoint;
        }

    private:
        Eigen::Vector3d worldPoint;
    };
}
