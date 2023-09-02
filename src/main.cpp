#include <iostream>

#include <Eigen/Dense>
#include <se3.hpp>
#include <opencv2/opencv.hpp>

#include "block_match.h"
#include "depth_filter.h"
#include "polar_search.h"
#include "read_dataset_files.h"


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

    for (std::size_t idx = 0; idx < imgFiles.size(); ++idx) {
        auto img = cv::imread(imgFiles[idx], cv::IMREAD_GRAYSCALE);
        if (img.data == nullptr)
            continue;
        const auto &world2CamPose = poseVec[idx];

        // todo
    }

    return 0;
}