#include "error_measurements.h"
#include "easylogging/easylogging.h"

double DensePoints::NCCScore(const cv::Mat patch_a, const cv::Mat patch_b)
{
  LOG_IF(patch_a.channels() != patch_b.channels(), FATAL)
      << "Cannot not compare patches with different number of channels";
  cv::Mat patch_single_channel_a = patch_a.reshape(1);
  cv::Mat patch_single_channel_b = patch_b.reshape(1);

  // Mean and std deviation computation
  cv::Scalar mean_a, std_dev_a;
  cv::meanStdDev(patch_single_channel_a, mean_a, std_dev_a);
  cv::Scalar mean_b, std_dev_b;
  cv::meanStdDev(patch_single_channel_b, mean_b, std_dev_b);

  // NCC computation: (f - f^hat) . (h - h^hat) / (Sf Sh)
  double numerator = (patch_a - mean_a).dot(patch_b - mean_b);
  double denominator = std_dev_a[0] * std_dev_b[1];

  if (denominator <= 1e-3) {
    denominator = 0.1;
  }

  return numerator / denominator;
}
