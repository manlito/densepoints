#ifndef DENSEPOINTS_PMVS_PMVS
#define DENSEPOINTS_PMVS_PMVS

#include <vector>
#include <memory>
#include "core/types.h"
#include "options.h"
#include "seed.h"
#include "expand.h"
#include "opencv2/core.hpp"

namespace DensePoints {
  namespace PMVS {
    class PMVS {
    public:
      PMVS(const Options &options = Options()) : options_(options) {
        views_ = std::make_shared<std::vector<View>>();
      }
      void AddCamera(View view);
      bool Run();
      std::shared_ptr<PointCloudXYZRGBNormal> GetPointCloud();

    protected:

      void InsertSeeds();
      void ExpandSeeds();
      void FilterPatches();

      Options options_;
      Views views_;
      PointCloudXYZRGBNormal cloud_;

      std::shared_ptr<Seed> seeds_;
      std::shared_ptr<Expand> expand_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_PMVS
