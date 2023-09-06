#include <fstream>
#include <sstream>

#include "read_dataset_files.h"


namespace dm
{
    /*
     * 1. 读取test_data中的images_pose.txt
     * 2. 读取文件中的位姿信息到poseVec中
     * 3. 读取文件中的图片路径到imgFiles中
     * 4. 读取test_data/depthmaps中的scene_000.depth到depth中
     * 5. 注意在进行depth写入时，需要将depth的单位进行统一为m
     * */
    void ReadDatasetFiles::operator()(const std::string &fp)
    {
        readPoses(fp);
        readDepth(fp);
    }

    void ReadDatasetFiles::readPoses(const std::string &fp)
    {
        std::string lineStr;
        std::ifstream ifsPose(fp + "/images_pose.txt");
        std::string imgPathSuffix;
        std::string imgPath = fp + "/images/";
        double data[7];
        while (std::getline(ifsPose, lineStr)) {
            std::istringstream line(lineStr);
            line >> imgPathSuffix;
            imgPath += imgPathSuffix;

            for (auto &item: data) {
                line >> item;
            }
            Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
            Eigen::Vector3d t(data[0], data[1], data[2]);
            Sophus::SE3d pose(q, t);

            imgFiles.push_back(imgPath);
            poseVec.push_back(std::move(pose));
            imgPath = fp + "/images/";
        }
    }

    void ReadDatasetFiles::readDepth(const std::string &fp)
    {
        std::ifstream ifsDepth(fp + "/depthmaps/scene_000.depth");
        double depthValue;

        for (int row = 0; row < HEIGHT; ++row) {
            for (int col = 0; col < WIDTH; ++col) {
                if (!(ifsDepth >> depthValue))
                    break;
                depthValue /= 100.0;
                depth.at<double>(row, col) = depthValue;
            }
        }
        cv::imshow("test", depth);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
}