#ifndef DENSEPOINTS_PMVS_FEATURES
#define DENSEPOINTS_PMVS_FEATURES

#include <vector>
#include <opencv2/core.hpp>
#include "features/matcher.h"
#include "patch.h"

namespace DensePoints {
  using namespace Features;
  namespace PMVS {
    struct SeedOptions {
      size_t patch_size;
      SeedOptions(size_t patch_size = 21) :
               patch_size(patch_size) {}
    };

    class Seed : public Matcher {
    public:
      Seed(Views views,
           MatcherOptions matcher_options = MatcherOptions(),
           SeedOptions seed_options = SeedOptions()) :
        Matcher(views, matcher_options),
        seed_options_(seed_options) {}

      void ConvertSeedsToPatches();
      void GetPatches(std::vector<Patch> &patches) { patches = patches_; }
    protected:

      void CreatePatchesFromPoints();

      // Optimization
      void OptimizeAndRefinePatches();
      void FilterPatches();
      void OptimizePatches();

      // Utility functions
      void RemovePatches(const std::vector<size_t> &patch_indices);
      void PrintPatches(const std::string folder_name);
      void PrintTextures(const std::string folder_name);

      std::vector<Patch> patches_;

      SeedOptions seed_options_;
    };

  }
}

#endif // DENSEPOINTS_PMVS_FEATURES
