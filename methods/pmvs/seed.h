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

    struct SeedOptions {
      DetectorType detector_type;
      MatcherType matcher_type;
      bool epipolar_matching;
      float max_epipolar_distance;
      size_t cell_size;
      size_t max_keypoints_per_cell;
      size_t patch_size;
      SeedOptions(DetectorType detector_type = DetectorType::ORB,
                  MatcherType matcher_type = MatcherType::kNN,
                  bool epipolar_matching = false,
                  float max_epipolar_distance = 1.5,
                  size_t cell_size = 32,
                  size_t max_keypoints_per_cell = 4,
                  size_t patch_size = 21) :
               detector_type(detector_type),
               matcher_type(matcher_type),
               epipolar_matching(epipolar_matching),
               max_epipolar_distance(max_epipolar_distance),
               cell_size(cell_size),
               max_keypoints_per_cell(max_keypoints_per_cell),
               patch_size(patch_size) {}
    };

    class Seed {
    public:
      Seed(Views views,
           SeedOptions options = SeedOptions()) :
        views_(views),
        options_(options) {}

      void GenerateSeeds();
      void GetPatches(std::vector<Patch> &patches) { patches = patches_; }
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

      // Optimization
      void OptimizeAndRefinePatches();
      void FilterPatches();
      void OptimizePatches();

      // Utility functions
      void RemovePatches(const std::vector<size_t> &patch_indices);
      void PrintPatches(const std::string folder_name);
      void PrintTextures(const std::string folder_name);

      Views views_;
      std::vector<std::vector<cv::KeyPoint>> keypoints_;
      std::vector<cv::Mat> descriptors_;
      ImagesPairsList pairs_list_;
      std::vector<std::vector<cv::DMatch>> matches_;
      std::vector<Vector3> points_;
      std::vector<Patch> patches_;

      SeedOptions options_;
    };

  }
}

#endif // DENSEPOINTS_PMVS_FEATURES
