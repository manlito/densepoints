#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "easylogging/easylogging.h"
#include "seed.h"
#include "core/grid.h"
#include "geometry/fundamental_matrix.h"
#include <omp.h>

using namespace DensePoints::PMVS;

void Seed::DetectKeypoints()
{
  keypoints_.resize(views_.size());
#pragma omp parallel for
  for (size_t view_index = 0; view_index < views_.size(); ++view_index) {

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

    LOG(INFO) << "Found " << keypoints.size() << " keypoints for image " << view_index;

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
    const size_t initial_keypoints = keypoints_[view_index].size();

    Grid grid(cell_size_, views_[view_index].GetImage().cols, views_[view_index].GetImage().rows);

    // Put each keypoint in its cell. We will keep only keypoint indexes
    std::vector<std::vector<size_t>> keypoints_grid(grid.Size());

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

    LOG(INFO) << "Reduced from "<< initial_keypoints << " to " << keypoints.size() << " keypoints for image index " << view_index;

#ifdef DEBUG_PMVS_SEEDS
    cv::Mat grayscale_image;
    cv::cvtColor(views_[view_index].GetImage(), grayscale_image, cv::COLOR_BGR2GRAY);
    cv::drawKeypoints(grayscale_image, keypoints, grayscale_image);
    cv::imwrite(std::string("kp_") + std::to_string(view_index) + std::string("_f.jpg"), grayscale_image);
    LOG(INFO) << "Best " << best_keypoints.size() << " keypoints for image index " << view_index;
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
  descriptors_.resize(views_.size());
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

void Seed::DefaultPairsList()
{
  pairs_list_.clear();

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
    pairs_list_.push_back(pair);
  } while (std::prev_permutation(image_pairs.begin(), image_pairs.end()));
}

void Seed::MatchKeypoints()
{
  matches_.resize(pairs_list_.size());
#pragma omp parallel for
  for (size_t match_index = 0; match_index < pairs_list_.size(); ++match_index) {
    const ImagesPair pair = pairs_list_[match_index];

    std::vector<cv::DMatch> matches;
    switch (matcher_type_) {
      case MatcherType::kNN: {
          cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
          const float nn_match_ratio = 0.7;
          std::vector<std::vector<cv::DMatch>> matches_groups;
          matcher->knnMatch(descriptors_[pair.first], descriptors_[pair.second], matches_groups, 2);
          for (size_t i = 0; i < matches_groups.size(); i++) {
              if (matches_groups[i][0].distance < nn_match_ratio * matches_groups[i][1].distance) {
                matches.push_back(matches_groups[i][0]);
              }
          }
          break;
        }
      case MatcherType::FLANN: {
          cv::FlannBasedMatcher matcher;
          matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(12, 20, 2));
          matcher.match(descriptors_[pair.first], descriptors_[pair.second], matches);

          std::vector<cv::DMatch> good_matches;
          for (size_t i = 0; i < matches.size(); i++) {
            if (matches[i].distance < 70) {
              good_matches.push_back(matches[i]);
            }
          }
          break;
        }
      default: LOG(FATAL) << "Unknown matcher type";
    }

    LOG(INFO) << "Found " << matches.size() << " matches for pair indices : " << pair.first << " " << pair.second;

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
        + std::to_string(pair.second) + std::string(".jpg"), output_image);
#endif

#pragma omp critical
    {
      matches_[match_index] = matches;
    }
  }
}

void Seed::FilterMatches()
{
#pragma omp parallel for
  for (size_t match_index = 0; match_index < pairs_list_.size(); ++match_index) {
    const ImagesPair &pair = pairs_list_[match_index];
    FundamentalMatrix fundamental_matrix = Geometry::ComputeFundamentalMatrix(
        views_[pair.first].projection_matrix,
        views_[pair.second].projection_matrix);
    float distanceSum = 0;

    // Matches vector indexes are in sync with pairs_list
    std::vector<cv::DMatch> matches = matches_[match_index];
    const size_t initial_matches = matches.size();
    std::vector<cv::DMatch>::iterator it = matches.begin();
    while (it != matches.end()) {
      const cv::DMatch &match = *it;
      const cv::KeyPoint &keypoint_left = keypoints_[pair.first][match.queryIdx];
      const cv::KeyPoint &keypoint_right = keypoints_[pair.second][match.trainIdx];

      Vector2 p1(keypoint_left.pt.x, keypoint_left.pt.y);
      Vector2 p2(keypoint_right.pt.x, keypoint_right.pt.y);
      Geometry::Line2D line = Geometry::LineFromFundamentalMatrix(fundamental_matrix, p1);
      float distance = line.distance(p2);

      if (distance > 5) {
        it = matches.erase(it);
      } else {
        distanceSum += distance;
        ++it;
      }
    }

    LOG(INFO) << "Reduced from " << initial_matches << " to " << matches.size() << " matches for pair indices : " << pair.first << " " << pair.second;

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

#pragma omp critical
    {
      matches_[match_index] = matches;
    }
  }
}

void Seed::GenerateSeeds(std::vector<Vector3> &seeds)
{
  // Detect keypoints
  DetectKeypoints();

  // Get the best keypoints
  FilterKeypoints();

  // Compute descriptors
  ComputeDescriptors();

  // Compute matches with a default pair list
  DefaultPairsList();
  MatchKeypoints();

  // Filter using distance to epipolar line
  FilterMatches();

  LOG(INFO) << "Done";
}
