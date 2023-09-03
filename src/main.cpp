#include <iostream>

#include <Eigen/Dense>
#include <se3.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include "config.h"
#include "block_match.h"
#include "depth_filter.h"
#include "polar_search.h"
#include "read_dataset_files.h"
#include "triangulation.h"

void oneIter(
        const std::vector<std::string> &imgFiles,
        const std::vector<Sophus::SE3d> &poseVec,
        cv::Mat &refImg, Sophus::SE3d &refWorld2CamPose,
        dm::PolarSearch &polarSearch, dm::BlockMatch &blockMatch,
        dm::Triangulation &triangulation, dm::DepthFilter &depthFilter
);

int main(int argc, const char **argv)
{
    if (argc != 2) {
        std::cout << "Usage: dense_mapping REMODE_data_file_path";
        return 1;
    }
    dm::ReadDatasetFiles rdf;
    rdf(argv[1]);
    const auto &imgFiles = rdf.getImgFiles();
    const auto &poseVec = rdf.getPoseVec();

    cv::Mat depth = rdf.getDepth();
    cv::Mat depthCov(depth.rows, depth.cols, CV_64F, 3.0);  // 将 3.0 填满depthCov; CV_64F代表双精度
    dm::PolarSearch polarSearch;
    dm::BlockMatch blockMatch(dm::WINDOW_SIZE);
    dm::Triangulation triangulation;
    dm::DepthFilter depthFilter(std::move(depth), std::move(depthCov));

    auto refImg = cv::imread(imgFiles[0], cv::IMREAD_GRAYSCALE);
    auto refWorld2CamPose = poseVec[0];
    std::string fileName;

    for (int i = 0; i < dm::ITERATIONS; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        oneIter(imgFiles, poseVec, refImg, refWorld2CamPose,
                polarSearch, blockMatch, triangulation, depthFilter);

        for (const Eigen::Vector3d &worldPoint: depthFilter.getWorldPoints()) {
            pcl::PointXYZ pclPoint;
            pclPoint.x = (float) worldPoint.x();
            pclPoint.y = (float) worldPoint.y();
            pclPoint.z = (float) worldPoint.z();
            cloud->push_back(pclPoint);
        }
        fileName = "iter_" + std::to_string(i) + "_result.ply";
        pcl::io::savePLYFile(fileName, *cloud);
    }

    return 0;
}

void oneIter(
        const std::vector<std::string> &imgFiles,
        const std::vector<Sophus::SE3d> &poseVec,
        cv::Mat &refImg, Sophus::SE3d &refWorld2CamPose,
        dm::PolarSearch &polarSearch, dm::BlockMatch &blockMatch,
        dm::Triangulation &triangulation, dm::DepthFilter &depthFilter
)
{
    for (std::size_t idx = 1; idx < imgFiles.size(); ++idx) {
        auto img = cv::imread(imgFiles[idx], cv::IMREAD_GRAYSCALE);
        if (img.data == nullptr)
            continue;
        const auto &world2CamPose = poseVec[idx];

        for (int row = dm::BORDER; row < dm::HEIGHT - dm::BORDER; ++row)
            for (int col = dm::BORDER; col < dm::WIDTH - dm::BORDER; ++col) {
                const Eigen::Vector2d refPixelPoint(row, col);
                polarSearch(refPixelPoint, refImg, img, refWorld2CamPose, world2CamPose);
                blockMatch(refImg, img, refPixelPoint, polarSearch.getPolarCoef());
                const Eigen::Vector2d &pixelPoint = blockMatch.getCurrPixelPoint();
                triangulation(refWorld2CamPose, world2CamPose, refPixelPoint, pixelPoint);
                depthFilter(refPixelPoint, triangulation.getWorldPoint(), refWorld2CamPose);
            }

        refImg = std::move(img);
        refWorld2CamPose = world2CamPose;
    }
}
