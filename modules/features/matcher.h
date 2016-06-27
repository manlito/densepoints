#ifndef DENSEPOINTS_FEATURES_MATCHER
#define DENSEPOINTS_FEATURES_MATCHER

#include <vector>
#include <opencv2/core.hpp>
#include "core/types.h"

namespace DensePoints {
  namespace Features {
    typedef std::pair<size_t, size_t> KeypointImagePair;
    enum class DetectorType { AKAZE, ORB };
    enum class MatcherType { FLANN, kNN };

    struct MatcherOptions {
      DetectorType detector_type;
      MatcherType matcher_type;
      bool epipolar_matching;
      float max_epipolar_distance;
      size_t cell_size;
      size_t max_keypoints_per_cell;
      MatcherOptions(DetectorType detector_type = DetectorType::ORB,
                  MatcherType matcher_type = MatcherType::kNN,
                  bool epipolar_matching = false,
                  float max_epipolar_distance = 1.5,
                  size_t cell_size = 16,
                  size_t max_keypoints_per_cell = 4) :
               detector_type(detector_type),
               matcher_type(matcher_type),
               epipolar_matching(epipolar_matching),
               max_epipolar_distance(max_epipolar_distance),
               cell_size(cell_size),
               max_keypoints_per_cell(max_keypoints_per_cell) {}
    };

    class Matcher {
    public:
      Matcher(Views views, MatcherOptions options = MatcherOptions()) :
        views_(views),
        options_(options) {}

      void GenerateSeeds();
    protected:

      void DetectKeypoints();
      void FilterKeypoints();
      void ComputeDescriptors();
      void DefaultPairsList();

      // Standard feature matching
      void MatchKeypoints();
      void FilterMatches();
      void GetAllMatches(KeypointImagePair &keypoint_index,
                         std::vector<KeypointImagePair> &keypoints_indices);
      void TriangulateMatches();

      // Matching using only distance to epipolar line
      void DirectEpipolarMatching();

      bool AddSeed(Vector3 X);

      Views views_;
      std::vector<std::vector<cv::KeyPoint>> keypoints_;
      std::vector<cv::Mat> descriptors_;
      ImagesPairsList pairs_list_;
      std::vector<std::vector<cv::DMatch>> matches_;
      std::vector<Vector3> points_;

      MatcherOptions options_;
    };

  }
}

#endif // DENSEPOINTS_FEATURES_MATCHER
