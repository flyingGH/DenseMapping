#pragma once

#include <vector>
#include <string>

#include <se3.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"

namespace dm
{
    class ReadDatasetFiles
    {
    public:
        ReadDatasetFiles() : depth(HEIGHT, WIDTH, CV_64F)
        {

        }

        void operator()(const std::string &fp);

        std::vector<std::string> &getImgFiles()
        {
            return imgFiles;
        }

        const std::vector<std::string> &getImgFiles() const
        {
            return imgFiles;
        }

        std::vector<Sophus::SE3d> &getPoseVec()
        {
            return poseVec;
        }

        const std::vector<Sophus::SE3d> &getPoseVec() const
        {
            return poseVec;
        }

        cv::Mat &getDepth()
        {
            return depth;
        }

        const cv::Mat &getDepth() const
        {
            return depth;
        }

    private:
        void readPoses(const std::string &fp);

        void readDepth(const std::string &fp);

        std::vector<std::string> imgFiles;
        std::vector<Sophus::SE3d> poseVec;
        cv::Mat depth;
    };

}