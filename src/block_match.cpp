#include "block_match.h"
#include "polar_search.h"
#include "config.h"

namespace dm
{
    /*
     * 1. 在polarSearch中的极线向量为初始极线向量
     * 2. 根据polarSearch中的极线向量和起始点来求解像素坐标系中极点的极线位置
     * 3. 根据求得极线起始点对极线向量进行更新
     * */
    bool BlockMatch::operator()(
            const cv::Mat &refImg, const cv::Mat &currImg,
            const Eigen::Vector2d &refPixelPoint,
            PolarSearch &polarSearch
    )
    {
        std::vector<double> refPatch, currPatch;
        Eigen::Vector2d pointStart = polarSearch.pointStart;
        Eigen::Vector2d pointEnd;
        // compute pointStart and pointEnd
        const double a = polarSearch.polarCoef.y();
        const double b = -polarSearch.polarCoef.x();
        const double c = -b * pointStart.y() - a * pointStart.x();

        if (a == 0) {
            double yValue = -c / b;
            pointStart = Eigen::Vector2d(BORDER, yValue);
            pointEnd = Eigen::Vector2d(WIDTH - BORDER, yValue);
            if (polarSearch.polarCoef.x() < 0)
                polarSearch.polarCoef *= -1;
        } else if (b == 0) {
            double xValue = -c / a;
            pointStart = Eigen::Vector2d(xValue, BORDER);
            pointEnd = Eigen::Vector2d(xValue, HEIGHT - BORDER);
            if (polarSearch.polarCoef.y() < 0)
                polarSearch.polarCoef *= -1;
        } else {
            Eigen::Vector2d pointLeft(BORDER, (-c - a * BORDER) / b);
            Eigen::Vector2d pointRight(WIDTH - BORDER, (-c - a * (WIDTH - BORDER)) / b);
            Eigen::Vector2d pointDown(-(c + b * (HEIGHT - BORDER)) / a, HEIGHT - BORDER);
            Eigen::Vector2d pointUp(-(c + b * BORDER) / a, BORDER);

            /*
             * 1. 如果a*b>0，pointDown和pointRight只能选一个，pointUp和pointLeft只能选一个
             * 2. 如果a*b<0，pointDown和pointLeft只能选一个，pointUp和pointRight只能选一个
             * */
            if (a * b > 0) {
                if (pointLeft.y() >= BORDER && pointLeft.y() <= HEIGHT - BORDER) {
                    pointStart = pointLeft;
                    if (pointRight.y() >= BORDER && pointRight.y() <= HEIGHT - BORDER)
                        pointEnd = pointRight;
                    else
                        pointEnd = pointDown;
                } else {
                    pointStart = pointUp;
                    if (pointRight.y() >= BORDER && pointRight.y() <= HEIGHT - BORDER)
                        pointEnd = pointRight;
                    else
                        pointEnd = pointDown;
                }
            } else {
                if (pointLeft.y() >= BORDER && pointLeft.y() <= HEIGHT - BORDER) {
                    pointStart = pointLeft;
                    if (pointRight.y() >= BORDER && pointRight.y() <= HEIGHT - BORDER)
                        pointEnd = pointRight;
                    else
                        pointEnd = pointUp;
                } else {
                    pointStart = pointDown;
                    if (pointRight.y() >= BORDER && pointRight.y() <= HEIGHT - BORDER)
                        pointEnd = pointRight;
                    else
                        pointEnd = pointUp;
                }
            }
            if (polarSearch.polarCoef.x() < 0)
                polarSearch.polarCoef *= -1;
        }

        const Eigen::Vector2d &polarCoef = polarSearch.polarCoef;

        Eigen::Vector2d point;
        double refMainPixel = 0.0;
        double patchNum = std::pow(2 * WINDOW_SIZE + 1, 2);
        for (int row = -WINDOW_SIZE; row <= WINDOW_SIZE; ++row) {
            point.y() = refPixelPoint.y() + row;
            for (int col = -WINDOW_SIZE; col <= WINDOW_SIZE; ++col) {
                point.x() = refPixelPoint.x() + col;
                double pixelValue = bilinear(refImg, point);
                refPatch.push_back(pixelValue);
                refMainPixel += pixelValue;
            }
            refMainPixel /= patchNum;
        }

        double maxSimValue = NCC_MATCH_THRESHOLD;
        for (; pointStart.x() <= pointEnd.x(); pointStart += polarSearch.polarCoef * POLAR_GAP_PIXEL) {
            double currMeanPixel = 0.0;
            for (int row = -WINDOW_SIZE; row <= WINDOW_SIZE; ++row) {
                point.y() = pointStart.y() + row;
                for (int col = -WINDOW_SIZE; col <= WINDOW_SIZE; ++col) {
                    point.x() = pointStart.x() + col;
                    double pixelValue = bilinear(currImg, pointStart);
                    currPatch.push_back(pixelValue);
                    currMeanPixel += pixelValue;
                }
                currMeanPixel /= patchNum;
            }
            // simValue越大，代表越相似，simValue越小代表越不相似
            double simValue = method(refPatch, currPatch, refMainPixel, currMeanPixel);
            if (simValue > maxSimValue) {
                maxSimValue = simValue;
                currPixelPoint = pointStart;
            }
        }
        if (maxSimValue > NCC_MATCH_THRESHOLD) {
            return true;
        }
        return false;
    }


    double BlockMatch::SAD(
            const std::vector<double> &refPatch,
            const std::vector<double> &currPatch,
            const double &refMeanPixel,
            const double &currMeanPixel
    )
    {
        const double deltaMean = refMeanPixel - currMeanPixel;
        double result = 0.0;
        for (std::size_t idx = 0; idx != refPatch.size(); ++idx) {
            result += std::abs(refPatch[idx] - currPatch[idx] - deltaMean);
        }
        return 1 / result;
    }

    double BlockMatch::SSD(const std::vector<double> &refPatch, const std::vector<double> &currPatch,
                           const double &refMeanPixel, const double &currMeanPixel)
    {
        const double deltaMean = refMeanPixel - currMeanPixel;
        double result = 0.0;
        for (std::size_t idx = 0; idx != refPatch.size(); ++idx) {
            result += std::pow(refPatch[idx] - currPatch[idx] - deltaMean, 2);
        }
        return 1 / result;
    }

    double BlockMatch::NCC(const std::vector<double> &refPatch, const std::vector<double> &currPatch,
                           const double &refMeanPixel, const double &currMeanPixel)
    {
        double SSDPatchA = 0.0;
        double SSDPatchB = 0.0;
        double SSDPatchAB = 0.0;

        for (std::size_t idx = 0; idx < refPatch.size(); ++idx) {
            SSDPatchA += std::pow(refPatch[idx] - refMeanPixel, 2);
            SSDPatchB += std::pow(currPatch[idx] - currMeanPixel, 2);

            SSDPatchAB += (refPatch[idx] - refMeanPixel) * (currPatch[idx] - currMeanPixel);
        }

        return SSDPatchAB / std::sqrt(SSDPatchA * SSDPatchB);
    }

    double BlockMatch::bilinear(const cv::Mat &img, const Eigen::Vector2d &point)
    {
        int pointIntX = static_cast<int>(point.x());
        int pointIntY = static_cast<int>(point.y());
        double deltaX = point.x() - pointIntX;
        double deltaY = point.y() - pointIntY;

        double pointIntValue = img.at<uchar>(pointIntY, pointIntX);
        double gradX = img.at<uchar>(pointIntY, pointIntX + 1) - pointIntValue;
        double gradY = img.at<uchar>(pointIntY + 1, pointIntX) - pointIntValue;

        return pointIntValue + deltaX * gradX + deltaY * gradY;
    }
}