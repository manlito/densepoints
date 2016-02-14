#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "easylogging/easylogging.h"
#include "seed.h"
#include "core/grid.h"

using namespace DensePoints::PMVS;

void Seed::DetectFeatures()
{
  while (current_job_ < views_.size()) {
    size_t job = current_job_++;
    LOG(INFO) << "Detecting keypoints for image index " << job;

    cv::Mat grayscale_image;
    cv::cvtColor(views_[job].GetImage(), grayscale_image, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints;

    switch (detector_type_) {
      case DetectorType::AKAZE: {
          cv::Ptr<cv::Feature2D> akaze = cv::AKAZE::create();
          akaze->detect(grayscale_image, keypoints);
          break;
        }
      default: {
          cv::Ptr<cv::Feature2D> fast = cv::ORB::create(40000);
          fast->detect(grayscale_image, keypoints);
        }
    }
    cv::drawKeypoints(grayscale_image, keypoints, grayscale_image);
    cv::imwrite(std::string("kp_") + std::to_string(job) + std::string(".jpg"), grayscale_image);
    LOG(INFO) << keypoints.size() << " Keypoints for image index " << job;

    // Save the result
    mutex_.lock();
    keypoints_[job] = keypoints;
    mutex_.unlock();
  }
}

void Seed::FilterKeypoints()
{
  while (current_job_ < views_.size()) {
    size_t job = current_job_++;
    LOG(INFO) << "Selecting best keypoints for image index " << job;

    Grid grid(cell_size_, views_[job].GetImage().cols, views_[job].GetImage().rows);

    // Put each keypoint in its cell. We will keep only keypoint indexes
    std::vector<std::vector<size_t>> keypoints_grid(grid.Size());
    LOG(INFO) << "Grid size for image index " << keypoints_grid.size();

    // Map each keypoint to the cell in the grid
    size_t index = 0;
    for (const cv::KeyPoint &keypoint : keypoints_[job]) {
      const size_t cell_xy = grid.CellXY(keypoint.pt.x, keypoint.pt.y);
      keypoints_grid[cell_xy].push_back(index);
      ++index;
    }

    // Select the best keypoints for each cell
    std::vector<size_t> best_keypoints;
    for (const std::vector<size_t> &cell : keypoints_grid) {
      std::vector<float> responses;
      for (const size_t &keypoint_index : cell) {
        responses.push_back(keypoints_[job][keypoint_index].response);
      }
      if (responses.size() > 0) {
        size_t sort_up_to = std::min(responses.size(), max_keypoints_per_cell_);

        // Get a list of top 4 responses, using the response index
        std::vector<size_t> indices(responses.size());
        for (size_t i = 0; i != indices.size(); ++i) indices[i] = i;
        if (responses.size() > max_keypoints_per_cell_) {
          std::nth_element(indices.begin(),
                           indices.begin() + sort_up_to,
                           indices.end(),
                           [&responses](size_t i1, size_t i2) { return responses[i1] > responses[i2]; });
        }
        for (size_t i = 0; i < sort_up_to; ++i) {
          best_keypoints.push_back(cell[indices[i]]);
        }
      }
    }
    std::vector<cv::KeyPoint> keypoints;
    for (const size_t &keypoint_index : best_keypoints) {
      keypoints.push_back(keypoints_[job][keypoint_index]);
    }

    // Debugging
    cv::Mat grayscale_image;
    cv::cvtColor(views_[job].GetImage(), grayscale_image, cv::COLOR_BGR2GRAY);
    cv::drawKeypoints(grayscale_image, keypoints, grayscale_image);
    cv::imwrite(std::string("kp_") + std::to_string(job) + std::string("_f.jpg"), grayscale_image);
    LOG(INFO) << "Best keypoints: " << best_keypoints.size();

    // Save the result
    mutex_.lock();
    keypoints_[job] = keypoints;
    mutex_.unlock();
  }
}

void Seed::GenerateSeeds(std::vector<Vector3> &seeds)
{

  // Reserve space where to put keypoints and descriptors
  keypoints_.resize(views_.size());
  descriptors_.resize(views_.size());

  // Detect keypoints
  current_job_ = 0;
  Threading::Run(thread_count_, &Seed::DetectFeatures, this);

  // Get the best keypoints
  current_job_ = 0;
  Threading::Run(thread_count_, &Seed::FilterKeypoints, this);

  LOG(INFO) << "Done";
}
