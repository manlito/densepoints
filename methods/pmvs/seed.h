#ifndef DENSEPOINTS_PMVS_FEATURES
#define DENSEPOINTS_PMVS_FEATURES

#include <vector>
#include <opencv2/core.hpp>
#include "patch.h"

namespace DensePoints {
  namespace PMVS {
    typedef std::pair<size_t, size_t> KeypointImagePair;
    enum class DetectorType { AKAZE, ORB };
    enum class MatcherType { FLANN, kNN };
    class Seed {
    public:
      Seed(std::vector<View> &views,
           DetectorType detector_type = DetectorType::ORB,
           MatcherType matcher_type = MatcherType::kNN,
           bool epipolar_matching = false,
           float max_epipolar_distance = 1.5,
           size_t cell_size = 32,
           size_t max_keypoints_per_cell = 4) :
        views_(views),
        detector_type_(detector_type),
        matcher_type_(matcher_type),
        epipolar_matching_(epipolar_matching),
        max_epipolar_distance_(max_epipolar_distance),
        cell_size_(cell_size),
        max_keypoints_per_cell_(max_keypoints_per_cell) {}

      void GenerateSeeds(std::vector<Vector3> &seeds);
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
      void CreatePatchesFromPoints();

      // Matching using only distance to epipolar line
      void DirectEpipolarMatching();

      bool AddSeed(Vector3 X);

      std::vector<View> &views_;
      std::vector<std::vector<cv::KeyPoint>> keypoints_;
      std::vector<cv::Mat> descriptors_;
      ImagesPairsList pairs_list_;
      std::vector<std::vector<cv::DMatch>> matches_;
      std::vector<Vector3> points_;
      std::vector<Patch> patches_;

      DetectorType detector_type_;
      MatcherType matcher_type_;
      float max_epipolar_distance_;
      bool epipolar_matching_;
      size_t cell_size_;
      size_t max_keypoints_per_cell_;
    };

  }
}

#endif // DENSEPOINTS_PMVS_FEATURES
