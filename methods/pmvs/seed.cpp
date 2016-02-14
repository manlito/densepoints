#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "easylogging/easylogging.h"
#include "seed.h"

using namespace DensePoints::PMVS;

void Seed::DetectFeatures()
{
  while (current_job_ < views_.size()) {
    size_t job = current_job_++;
    LOG(INFO) << "Detecting keypoints for image index " << job;

    cv::Mat grayscale_image;
    cv::cvtColor(views_[job].GetImage(), grayscale_image, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    switch (detector_type_) {
      case DetectorType::AKAZE: {
          cv::Ptr<cv::Feature2D> akaze = cv::AKAZE::create();
          akaze->detectAndCompute(grayscale_image, cv::Mat(), keypoints, descriptors);
          break;
        }
      default: {
          cv::Ptr<cv::Feature2D> fast = cv::ORB::create(20000);
          fast->detectAndCompute(grayscale_image, cv::Mat(), keypoints, descriptors);
        }
    }
    LOG(INFO) << keypoints.size() << " Keypoints for image index " << job;

    // Save the result
    mutex_.lock();
    keypoints_[job] = keypoints;
    descriptors_[job] = descriptors;
    mutex_.unlock();
  }
}

void Seed::GenerateSeeds(std::vector<Vector3> &seeds)
{
  // Detect keypoints
  current_job_ = 0;

  // Reserve space where to put keypoints and descriptors
  keypoints_.resize(views_.size());
  descriptors_.resize(views_.size());

  for (size_t i = 0; i < thread_count_; ++i) {
    threads_.push_back(std::thread(&Seed::DetectFeatures, this));
  }
  for (std::thread &thread : threads_) {
    thread.join();
  }
  LOG(INFO) << "Done";
}
