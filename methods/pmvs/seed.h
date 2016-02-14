#ifndef DENSEPOINTS_PMVS_FEATURES
#define DENSEPOINTS_PMVS_FEATURES

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/core.hpp>
#include "patch.h"

namespace DensePoints {
  namespace PMVS {
    enum class DetectorType { AKAZE, ORB };
    class Seed {
    public:
      Seed(std::vector<View> &views,
           size_t thread_count = 6,
           DetectorType detector_type = DetectorType::ORB,
           size_t cell_size = 32,
           size_t max_keypoints_per_cell = 2) :
        views_(views),
        detector_type_(detector_type),
        thread_count_(thread_count),
        cell_size_(cell_size),
        max_keypoints_per_cell_(max_keypoints_per_cell) {}

      void GenerateSeeds(std::vector<Vector3> &seeds);
    protected:
      // Thread facility
      std::vector<std::thread> threads_;
      std::atomic<size_t> current_job_;
      std::mutex mutex_;

      void DetectFeatures();
      void FilterKeypoints();

      std::vector<View> &views_;
      std::vector<std::vector<cv::KeyPoint>> keypoints_;
      std::vector<cv::Mat> descriptors_;

      DetectorType detector_type_;
      size_t thread_count_;
      size_t cell_size_;
      size_t max_keypoints_per_cell_;
    };

  }
}

#endif // DENSEPOINTS_PMVS_FEATURES
