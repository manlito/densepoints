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
      PMVS(const Options &options) : options_(options) {}
      void AddCamera(const ProjectionMatrix &projection_matrix,
                     const std::string filename);
      bool Run();
      std::shared_ptr<PointCloudXYZRGBNormal> GetPointCloud();

    protected:

      void InsertSeeds();
      void Expand();
      void Filter();

      Options options_;
      std::vector<ProjectionMatrix> projection_matrices_;
      std::vector<cv::Mat> images_;
      PointCloudXYZRGBNormal cloud_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_PMVS
