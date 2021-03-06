#ifndef DENSEPOINTS_ERROR_MEASUREMENTS
#define DENSEPOINTS_ERROR_MEASUREMENTS

#include <opencv2/core.hpp>

namespace DensePoints {

inline void ToFloatMat(const cv::Mat patch, cv::Mat &patch_float);

double NCCScoreByChannel(const cv::Mat patch_a, const cv::Mat patch_b);
double NCCScore(const cv::Mat patch_a, const cv::Mat patch_b);
double SSDScore(const cv::Mat patch_a, const cv::Mat patch_b);

}

#endif // DENSEPOINTS_ERROR_MEASUREMENTS
