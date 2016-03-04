#include <algorithm>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/features2d.hpp>
#include "easylogging/easylogging.h"
#include "seed.h"
#include "core/grid.h"
#include "geometry/fundamental_matrix.h"
#include "geometry/triangulation.h"
#include "rplycpp/src/rplycpp.hpp"

using namespace DensePoints::PMVS;

void Seed::GenerateSeeds(std::vector<Vector3> &seeds)
{
  // Detect keypoints
  DetectKeypoints();

  // Get the best keypoints
  FilterKeypoints();

  // Compute matches with a default pair list
  DefaultPairsList();

  if (!epipolar_matching_) {
    // Compute descriptors
    ComputeDescriptors();
    // Match with Flann or Bruteforce
    MatchKeypoints();
    // Filter using distance to epipolar line
    FilterMatches();
  } else {
    // Accept matches using only distance to epipolar line
    DirectEpipolarMatching();
  }

  // Triangulate
  TriangulateMatches();

  LOG(INFO) << "Done";
}

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
            if (matches[i].distance < 30) {
              good_matches.push_back(matches[i]);
            }
          }
          matches = good_matches;
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

void Seed::DirectEpipolarMatching()
{
  matches_.resize(pairs_list_.size());
#pragma omp parallel for
  for (size_t match_index = 0; match_index < pairs_list_.size(); ++match_index) {
    const ImagesPair pair = pairs_list_[match_index];
    FundamentalMatrix fundamental_matrix = Geometry::ComputeFundamentalMatrix(
        views_[pair.first].GetProjectionMatrix(),
        views_[pair.second].GetProjectionMatrix());

    std::vector<cv::DMatch> matches;
    // For each keypoint in the left image
    size_t queryIdx = 0;
    for (const cv::KeyPoint &keypoint_left : keypoints_[pair.first]) {
      const Vector2 point_left(keypoint_left.pt.x, keypoint_left.pt.y);
      // Look in the right points that lie in the epipolar line
      size_t trainIdx = 0;
      for (const cv::KeyPoint &keypoint_right : keypoints_[pair.second]) {
        const Vector2 point_right(keypoint_right.pt.x, keypoint_right.pt.y);
        Geometry::Line2D line = Geometry::LineFromFundamentalMatrix(fundamental_matrix, point_left);
        float distance = line.distance(point_right);
        if (distance <= max_epipolar_distance_) {
          matches.push_back(cv::DMatch(queryIdx, trainIdx, distance));
        }
        ++trainIdx;
      }
      ++queryIdx;
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
        views_[pair.first].GetProjectionMatrix(),
        views_[pair.second].GetProjectionMatrix());
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

      if (distance > max_epipolar_distance_) {
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

void Seed::GetAllMatches(KeypointImagePair &keypoint_index,
                         std::vector<KeypointImagePair> &keypoints_indices)
{
  for (size_t match_index = 0; match_index < pairs_list_.size(); ++match_index) {
    const ImagesPair &pair = pairs_list_[match_index];
    // Check if the pair contains the image index, as with the query keypoint
    bool compare_query_idx;
    if (pair.first == keypoint_index.first) {
      compare_query_idx = true;
    } else if (pair.second == keypoint_index.first) {
      compare_query_idx = false;
    } else {
      continue;
    }
    // Look for a match in the image index, with the query keypoint
    // We need to find in vector<DMatch>, for queryIdx or trainIdx
    // that matches our input keypoint
    KeypointImagePair candidate_keypoint_index;
    bool match_found = false;
    const std::vector<cv::DMatch> &matches = matches_[match_index];
    for (const cv::DMatch &match : matches) {
      if (compare_query_idx && match.queryIdx == keypoint_index.second) {
        // When is to left of DMatch, we want the data on the train
        candidate_keypoint_index.first = pair.second;
        candidate_keypoint_index.second = match.trainIdx;
        match_found = true;
        break;
      } else if (!compare_query_idx && match.trainIdx == keypoint_index.second) {
        // When is to right of DMatch, we want the data on the query
        candidate_keypoint_index.first = pair.first;
        candidate_keypoint_index.second = match.queryIdx;
        match_found = true;
        break;
      }
    }
    if (match_found) {
      keypoints_indices.push_back(candidate_keypoint_index);
    }
  }
}

void Seed::TriangulateMatches()
{
  for (size_t view_index = 0; view_index < views_.size(); ++view_index) {
    const std::vector<cv::KeyPoint> &keypoints = keypoints_[view_index];
#pragma omp parallel for
    for (size_t keypoint_index = 0; keypoint_index < keypoints.size(); ++keypoint_index) {
      KeypointImagePair keypoint_image_pair;
      keypoint_image_pair.first = view_index;
      keypoint_image_pair.second = keypoint_index;
      std::vector<KeypointImagePair> keypoint_image_pairs;
      // Find all matches that include this keypoint
      GetAllMatches(keypoint_image_pair, keypoint_image_pairs);

      // Make sure at least 1 match is found
      if (keypoint_image_pairs.size() >= 1) {
        std::vector<ProjectionMatrix> projection_matrices;
        std::vector<Vector2> observations;
        projection_matrices.push_back(views_[view_index].GetProjectionMatrix());
        cv::Point point = keypoints_[view_index][keypoint_index].pt;
        observations.push_back(Vector2(point.x, point.y));
        // Add the current view
        for (KeypointImagePair matched_keypoint_image_pair : keypoint_image_pairs) {
          const size_t matched_view_index = matched_keypoint_image_pair.first;
          projection_matrices.push_back(views_[matched_view_index].GetProjectionMatrix());
          point = keypoints_[matched_view_index][matched_keypoint_image_pair.second].pt;
          observations.push_back(Vector2(point.x, point.y));
        }
        // Triangulate
        Vector3 X = Geometry::DirectLinearTriangulation(projection_matrices, observations);
#pragma omp critical
        {
          points_.push_back(X);
        }
      }
    }
  }

//#ifdef DEBUG_PMVS_SEEDS
  // Debug triangulation
  rplycpp::PLYWriter writer;
  writer.Open("triangulation.ply");
  std::vector<rplycpp::PLYProperty> properties;
  rplycpp::PLYProperty property;
  property.type = rplycpp::PLYDataType::PLY_FLOAT;
  property.name = "x";
  properties.push_back(property);
  property.name = "y";
  properties.push_back(property);
  property.name = "z";
  properties.push_back(property);
  writer.AddElement("vertex", properties, points_.size());

  // Add the data IN THE SAME ORDER!
  for (const Vector3 &point : points_) {
    writer.AddRow(std::vector<double> {point[0], point[1], point[2]});
  }
  writer.Close();
//#endif

}
