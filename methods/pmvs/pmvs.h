#ifndef DENSEPOINTS_PMVS_PMVS
#define DENSEPOINTS_PMVS_PMVS

#include <vector>
#include <memory>
#include "core/types.h"
#include "options.h"
#include "opencv2/core.hpp"

namespace DensePoints {
  namespace PMVS {
    class PMVS {
    public:
      PMVS(const Options &options = Options()) : options_(options) {}
      void AddCamera(View view);
      bool Run();
      std::shared_ptr<PointCloudXYZRGBNormal> GetPointCloud();

    protected:

      void InsertSeeds();
      void Expand();
      void Filter();

      Options options_;
      std::vector<View> views_;
      PointCloudXYZRGBNormal cloud_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_PMVS
