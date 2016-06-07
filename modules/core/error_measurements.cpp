#include <opencv2/imgproc.hpp>
#include "error_measurements.h"

inline void DensePoints::ToFloatMat(const cv::Mat patch, cv::Mat &patch_float)
{
  // Convert to gray scale if necessary
  cv::Mat patch_grayscale;
  if (patch.type() == CV_8UC3) {
    cv::cvtColor(patch, patch_grayscale, CV_BGR2GRAY);
  } else {
    patch_grayscale = patch;
  }

  // Convert to float patches, to support negative values
  patch_grayscale.convertTo(patch_float, CV_32F);
}

double DensePoints::SSDScore(cv::Mat patch_a, cv::Mat patch_b)
{
  if (patch_a.empty() || patch_b.empty()) {
    return -1;
  }

  // Conver to float
  cv::Mat patch_float_a, patch_float_b;
  ToFloatMat(patch_a, patch_float_a);
  ToFloatMat(patch_b, patch_float_b);

  size_t patch_size = patch_a.cols * patch_a.rows;
  // SSD computation: (f - h)
  cv::Mat error = patch_float_a - patch_float_b;
  double sum = cv::sum(error.mul(error))[0];
  return sum / static_cast<double>(patch_size);
}

double DensePoints::NCCScore(cv::Mat patch_a, cv::Mat patch_b)
{
  if (patch_a.empty() || patch_b.empty()) {
    return -1;
  }

  // Conver to float
  cv::Mat patch_float_a, patch_float_b;
  ToFloatMat(patch_a, patch_float_a);
  ToFloatMat(patch_b, patch_float_b);

  size_t patch_size = patch_a.cols * patch_a.rows;
  // NCC computation: (f - f^hat) . (h - h^hat) / (Sf Sh)
  cv::Scalar mean_a, std_dev_a;
  cv::meanStdDev(patch_float_a, mean_a, std_dev_a);
  cv::Scalar mean_b, std_dev_b;
  cv::meanStdDev(patch_float_b, mean_b, std_dev_b);

  double numerator = (patch_float_a - mean_a[0]).dot(patch_float_b - mean_b[0]);
  double denominator = std_dev_a[0] * std_dev_b[0];

  denominator = std::max(1e-1, denominator);
  double ncc = (numerator / denominator) / static_cast<double>(patch_size);
  return ncc;
}

double DensePoints::NCCScoreByChannel(cv::Mat patch_a, cv::Mat patch_b)
{
  if (patch_a.empty() || patch_b.empty()) {
    return -1;
  }

  // Conver to float patches, to support negative values
  cv::Mat patch_float_a;
  patch_a.convertTo(patch_float_a, CV_32F);
  //patch_float_a /= 255.0;
  cv::Mat patch_float_b;
  patch_b.convertTo(patch_float_b, CV_32F);
  //patch_float_b /= 255.0;

  // Split channels
  std::vector<cv::Mat> channels_a(patch_a.channels());
  std::vector<cv::Mat> channels_b(patch_b.channels());
  cv::split(patch_float_a, channels_a);
  cv::split(patch_float_b, channels_b);

  // NCC computation: (f - f^hat) . (h - h^hat) / (Sf Sh)
  double average_ncc = 0;
  size_t patch_size = patch_a.cols * patch_a.rows;
  for (size_t i = 0; i < 3; ++i) {

    cv::Scalar mean_a, std_dev_a;
    cv::meanStdDev(channels_a[i], mean_a, std_dev_a);
    cv::Scalar mean_b, std_dev_b;
    cv::meanStdDev(channels_b[i], mean_b, std_dev_b);

    double numerator = (channels_a[i] - mean_a[0]).dot(channels_b[i] - mean_b[0]);
    double denominator = std_dev_a[0] * std_dev_b[0];

    denominator = std::max(1e-3, denominator);
    average_ncc += numerator / denominator;
  }

  return average_ncc / (patch_size * 3);
}
