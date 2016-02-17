#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "easylogging/easylogging.h"
#include "seed.h"
#include "core/grid.h"
#include <omp.h>

using namespace DensePoints::PMVS;

void Seed::DetectKeypoints()
{
#pragma omp parallel for
  for (size_t view_index = 0; view_index < views_.size(); ++view_index) {
    LOG(INFO) << "Detecting keypoints for image " << view_index;

    cv::Mat image;
    views_[view_index].GetImage().copyTo(image);
    std::vector<cv::KeyPoint> keypoints;

    switch (detector_type_) {
      case DetectorType::AKAZE: {
          cv::Ptr<cv::Feature2D> akaze = cv::AKAZE::create();
          akaze->detect(image, keypoints);
          break;
        }
      default: {
          cv::Ptr<cv::Feature2D> fast = cv::ORB::create(40000);
          fast->detect(image, keypoints);
        }
    }

#ifdef DEBUG_PMVS_SEEDS
    cv::Mat grayscale_image;
    cv::cvtColor(views_[view_index].GetImage(), grayscale_image, cv::COLOR_BGR2GRAY);
    cv::drawKeypoints(image, keypoints, image);
    cv::imwrite(std::string("kp_") + std::to_string(view_index) + std::string(".jpg"), grayscale_image);
    LOG(INFO) << keypoints.size() << " Keypoints for image index " << view_index;
#endif

#pragma omp critical
    {
      // Save the result
      keypoints_[view_index] = keypoints;
    }
  }
}

void Seed::FilterKeypoints()
{
#pragma omp parallel for
  for (size_t view_index = 0; view_index < views_.size(); ++view_index) {
    LOG(INFO) << "Selecting best keypoints for image index " << view_index;

    Grid grid(cell_size_, views_[view_index].GetImage().cols, views_[view_index].GetImage().rows);

    // Put each keypoint in its cell. We will keep only keypoint indexes
    std::vector<std::vector<size_t>> keypoints_grid(grid.Size());
    LOG(INFO) << "Grid size for image index " << keypoints_grid.size();

    // Map each keypoint to the cell in the grid
    size_t index = 0;
    for (const cv::KeyPoint &keypoint : keypoints_[view_index]) {
      const size_t cell_xy = grid.CellXY(keypoint.pt.x, keypoint.pt.y);
      keypoints_grid[cell_xy].push_back(index);
      ++index;
    }

    // Select the best keypoints for each cell
    std::vector<size_t> best_keypoints;
    for (const std::vector<size_t> &cell : keypoints_grid) {
      std::vector<float> responses;
      for (const size_t &keypoint_index : cell) {
        responses.push_back(keypoints_[view_index][keypoint_index].response);
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
      keypoints.push_back(keypoints_[view_index][keypoint_index]);
    }

#ifdef DEBUG_PMVS_SEEDS
    cv::Mat grayscale_image;
    cv::cvtColor(views_[view_index].GetImage(), grayscale_image, cv::COLOR_BGR2GRAY);
    cv::drawKeypoints(grayscale_image, keypoints, grayscale_image);
    cv::imwrite(std::string("kp_") + std::to_string(view_index) + std::string("_f.jpg"), grayscale_image);
    LOG(INFO) << "Best keypoints: " << best_keypoints.size();
#endif

#pragma omp critical
    {
      // Save the result
      keypoints_[view_index] = keypoints;
    }
  }
}

void Seed::ComputeDescriptors()
{
#pragma omp parallel for
  for (size_t view_index = 0; view_index < views_.size(); ++view_index) {
    LOG(INFO) << "Computing descriptors for image index " << view_index;

    cv::Mat &image = views_[view_index].GetImage();
    std::vector<cv::KeyPoint> &keypoints = keypoints_[view_index];
    cv::Mat descriptors;
    switch (detector_type_) {
      case DetectorType::AKAZE: {
          cv::Ptr<cv::Feature2D> akaze = cv::AKAZE::create();
          akaze->compute(image, keypoints, descriptors);
          break;
        }
      default: {
          cv::Ptr<cv::Feature2D> fast = cv::ORB::create();
          fast->compute(image, keypoints, descriptors);
        }
    }

#pragma omp critical
    {
      // Save the result
      descriptors_[view_index] = descriptors;
    }
  }
}

void Seed::BuilPairsList(ImagesPairsList &pairs_list)
{
  pairs_list.clear();

  // Used to generate a combination of pairs (0-1, 0-2, 0-3, 1-2. 1-3...)
  std::vector<bool> image_pairs(views_.size());
  std::fill(image_pairs.begin(), image_pairs.end() - views_.size() + 2, true);
  do {
    std::vector<size_t> indices;
    for (size_t i = 0; i < views_.size(); ++i) {
      if (image_pairs[i]) {
        indices.push_back(i);
      }
    }
    ImagesPair pair;
    pair.first = indices[0];
    pair.second = indices[1];
    pairs_list.push_back(pair);
  } while (std::prev_permutation(image_pairs.begin(), image_pairs.end()));
}

void Seed::MatchKeypoints(const ImagesPairsList &pairs_list)
{
#pragma omp parallel for
  for (size_t index = 0; index < pairs_list.size(); ++index) {
    const ImagesPair pair = pairs_list[index];
    LOG(INFO) << "Matching for pair indices : " << pair.first << " " << pair.second;

    std::vector<cv::DMatch> matches;
    cv::FlannBasedMatcher matcher;
    matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(12, 20, 2));
    matcher.match(descriptors_[pair.first], descriptors_[pair.second], matches);

#ifdef DEBUG_PMVS_SEEDS
    cv::Mat grayscale_image_1;
    cv::Mat grayscale_image_2;
    cv::Mat output_image;
    cv::cvtColor(views_[pair.first].GetImage(), grayscale_image_1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(views_[pair.second].GetImage(), grayscale_image_2, cv::COLOR_BGR2GRAY);
    cv::drawMatches(grayscale_image_1, keypoints_[pair.first],
        grayscale_image_2, keypoints_[pair.second],
        matches, output_image);
    cv::imwrite(std::string("matches_") + std::to_string(pair.first) + "_"
        + std::to_string(pair.second) + std::string("_f.jpg"), output_image);
#endif

//#pragma omp critical
//      {
//        //descriptors_[job] = descriptors;
//      }
  }
}

void Seed::GenerateSeeds(std::vector<Vector3> &seeds)
{

  // Reserve space where to put keypoints and descriptors
  keypoints_.resize(views_.size());
  descriptors_.resize(views_.size());

  // Detect keypoints
  DetectKeypoints();

  // Get the best keypoints
  FilterKeypoints();

  // Compute descriptors
  ComputeDescriptors();

  // Compute matches with a default pair list
  ImagesPairsList pairs_list;
  BuilPairsList(pairs_list);
  MatchKeypoints(pairs_list);

  LOG(INFO) << "Done";
}
