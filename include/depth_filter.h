#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <se3.hpp>

namespace dm
{

    class DepthFilter
    {
    public:
        using WorldPoints = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

        DepthFilter() = delete;

        DepthFilter(cv::Mat depth, cv::Mat depthConv) : depth(std::move(depth)), depthConv(std::move(depthConv))
        {

        }

        void operator()(
                const Eigen::Vector2d &refPoint,
                const Eigen::Vector3d &worldPoint,
                const Sophus::SE3d &refPose
        );

        cv::Mat &getDepth()
        {
            return depth;
        }

        const cv::Mat &getDepth() const
        {
            return depth;
        }

        WorldPoints &getWorldPoints()
        {
            return worldPoints;
        }

        const WorldPoints &getWorldPoints() const
        {
            return worldPoints;
        }

    private:
        cv::Mat depth;
        cv::Mat depthConv;
        WorldPoints worldPoints;
    };
}

