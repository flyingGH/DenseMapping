#include "polar_search.h"
#include "config.h"

namespace dm
{
    /*
     * 1. 在世界坐标系中，求得向量O1P的方向
     * 2. 根据深度收敛范围，对P点位置进行限定
     * 3. 根据P点的极限位置进行投影确定极线方向和限定的极线位置
     * 4. 限定的极线位置用来减少不必要的计算
     * */
    void PolarSearch::operator()(
            const Eigen::Vector2d &refPoint,
            const Sophus::SE3d &refPose,
            const Sophus::SE3d &currPose
    )
    {
        // 1. 在世界坐标系中，求得向量O1P的方向
        double x = (refPoint.x() - C_X) / F_X;
        double y = (refPoint.y() - C_Y) / F_Y;
        Eigen::Vector3d O1P = refPose.so3() * Eigen::Vector3d(x, y, 1);
        O1P.normalize();

        // 2.根据深度收敛范围，对P点位置进行限定
        Eigen::Vector3d worldPMax = refPose.translation() + (DEPTH_MAX * O1P);
        Eigen::Vector3d worldPMin = refPose.translation() + (DEPTH_MIN * O1P);

        // 3. 根据P点的极限位置进行投影确定极线方向和限定的极线位置
        Eigen::Vector3d currCamPointMax = currPose.inverse() * worldPMax;
        Eigen::Vector3d currCamPointMin = currPose.inverse() * worldPMin;
        double zMax = currCamPointMax.z();
        double zMin = currCamPointMin.z();
        pointEnd.x() = currCamPointMax.x() / zMax * F_X + C_X;
        pointEnd.y() = currCamPointMax.y() / zMax * F_Y + C_Y;
        pointStart.x() = currCamPointMin.x() / zMin * F_X + C_X;
        pointStart.y() = currCamPointMin.y() / zMin * F_Y + C_Y;
        polarCoef = pointEnd - pointStart;
        polarCoef.normalize();
    }
}