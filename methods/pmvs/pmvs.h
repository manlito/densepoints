#ifndef DENSEPOINTS_PMVS_PMVS
#define DENSEPOINTS_PMVS_PMVS

#include <vector>
#include "modules/core/types.h"
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
      void GetPointCloud(std::vector<Point3Normal> &points_with_normal,
                         std::vector<Color> &colors);

    protected:
      Options options_;
      std::vector<ProjectionMatrix> projection_matrices_;
      std::vector<cv::Mat> images_;
    };
  }
}

#endif DENSEPOINTS_PMVS_PMVS
