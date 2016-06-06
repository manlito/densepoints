#include <opencv2/imgproc.hpp>
#include "error_measurements.h"
#include "easylogging/easylogging.h"

double DensePoints::SSDScore(cv::Mat patch_a, cv::Mat patch_b)
{
  if (patch_a.empty() || patch_b.empty()) {
    LOG(WARNING) << "Trying to compare with empty image";
    return -1;
  }

  // Conver to float patches, to support negative values
  cv::Mat patch_grayscale_a, patch_grayscale_b;
  cv::cvtColor(patch_a, patch_grayscale_a, CV_BGR2GRAY);
  cv::cvtColor(patch_b, patch_grayscale_b, CV_BGR2GRAY);
  cv::Mat patch_float_a, patch_float_b;
  patch_grayscale_a.convertTo(patch_float_a, CV_32F);
  patch_grayscale_b.convertTo(patch_float_b, CV_32F);
  patch_float_a /= 255.0;
  patch_float_b /= 255.0;

  size_t patch_size = patch_a.cols * patch_a.rows;
  // SSD computation: (f - h)
  cv::Mat error = patch_float_a - patch_float_b;
  double sum = cv::sum(error.mul(error))[0];
  return sum / static_cast<double>(patch_size);
}

double DensePoints::NCCScore(cv::Mat patch_a, cv::Mat patch_b)
{
  if (patch_a.empty() || patch_b.empty()) {
    LOG(WARNING) << "Trying to compare with empty image";
    return -1;
  }

  // Conver to float patches, to support negative values
  cv::Mat patch_grayscale_a, patch_grayscale_b;
  cv::cvtColor(patch_a, patch_grayscale_a, CV_BGR2GRAY);
  cv::cvtColor(patch_b, patch_grayscale_b, CV_BGR2GRAY);
  cv::Mat patch_float_a, patch_float_b;
  patch_grayscale_a.convertTo(patch_float_a, CV_32F);
  patch_grayscale_b.convertTo(patch_float_b, CV_32F);

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
    LOG(WARNING) << "Trying to compare with empty image";
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
