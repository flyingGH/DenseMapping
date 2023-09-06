#include "triangulation.h"
#include "config.h"

namespace dm
{
    /*
     * 1. 求向量O1P1在世界坐标系中的方向
     * 2. 求向量O2P2在世界坐标系中的方向
     * 3. 根据正弦定理求得向量O1P1的模长
     * 4. 根据模长和相机中心位置求得世界坐标系上的点
     * */
    void Triangulation::operator()(
            const Sophus::SE3d &refPose, const Sophus::SE3d &currPose,
            const Eigen::Vector2d &refPoint, const Eigen::Vector2d &currPoint
    )
    {
        Eigen::Vector3d deltaT = currPose.translation() - refPose.translation();
        Eigen::Vector3d O1P1RefCam((refPoint.x() - C_X) / F_X, (refPoint.y() - C_Y) / F_Y, 1);
        Eigen::Vector3d O2P2CurrCam((currPoint.x() - C_X) / F_X, (currPoint.y() - C_Y) / F_Y, 1);
        Eigen::Vector3d O1P1World = refPose.so3() * O1P1RefCam;
        Eigen::Vector3d O2P2World = currPose.so3() * O2P2CurrCam;
        O1P1World.normalize();
        O2P2World.normalize();

        double cosGamma = O1P1World.dot(O2P2World) / O2P2World.norm();
        double cosBeta = -deltaT.dot(O2P2World) / deltaT.norm();
        double gamma = std::acos(cosGamma);
        double beta = std::acos(cosBeta);

        double O1P1Length = deltaT.norm() * std::sin(beta) / std::sin(gamma);
        worldPoint = refPose.translation() + O1P1Length * O1P1World;
    }
}