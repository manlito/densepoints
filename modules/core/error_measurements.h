#ifndef DENSEPOINTS_ERROR_MEASUREMENTS
#define DENSEPOINTS_ERROR_MEASUREMENTS

#include <opencv2/core.hpp>

namespace DensePoints {

double NCCScoreByChannel(const cv::Mat patch_a, const cv::Mat patch_b);
double NCCScore(const cv::Mat patch_a, const cv::Mat patch_b);
double SSDScore(const cv::Mat patch_a, const cv::Mat patch_b);

}

#endif // DENSEPOINTS_ERROR_MEASUREMENTS
