#include "depth_filter.h"
#include "triangulation.h"
#include "config.h"

namespace dm
{
    /*
     * 1. 判断深度是否收敛
     * 2. 如果深度收敛，则更新预测深度和预测标准差
     * 3. 如果深度收敛，根据更新后的深度对世界坐标系中的观测点进行更新，放入worldPoints中
     * */
    void DepthFilter::operator()(
            const Eigen::Vector2d &refPoint,
            const Eigen::Vector3d &worldPoint,
            const Sophus::SE3d &refPose,
            const double &sigmaV
    )
    {
        const double sigmaW = depthConv.at<double>((int) refPoint.y(), (int) refPoint.x());
        const double dp = depth.at<double>((int) refPoint.y(), (int) refPoint.x());
        const double dm = (worldPoint - refPose.translation()).norm();

        const double sigmaV2 = std::pow(sigmaV, 2);
        const double sigmaW2 = std::pow(sigmaW, 2);

        const double updateDepth = (sigmaV2 * dp + sigmaW2 * dm) / (sigmaV2 + sigmaW2);
        const double updateConv = std::sqrt((sigmaV2 * sigmaW2) / (sigmaV2 + sigmaW2));

        if (updateDepth <= DEPTH_MAX && updateDepth >= DEPTH_MIN) {
            depth.at<double>((int) refPoint.y(), (int) refPoint.x()) = updateDepth;
            depthConv.at<double>((int) refPoint.y(), (int) refPoint.x()) = updateConv;
            Eigen::Vector3d updateWorldPoint =
                    (updateDepth / dm) * (worldPoint - refPose.translation()) + refPose.translation();
            worldPoints.emplace_back(std::move(updateWorldPoint));
        }
    }
}