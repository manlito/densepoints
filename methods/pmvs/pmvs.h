#ifndef DENSEPOINTS_PMVS_PMVS
#define DENSEPOINTS_PMVS_PMVS

#include <vector>
#include "modules/geometry/types.h"
#include "options.h"

namespace DensePoints {
  namespace PMVS {
    class PMVS {
    public:
      PMVS(const Options &options) : options_(options) {}
      void AddCamera(const ProjectionMatrix &projection_matrix);
      bool Run();
      void GetPointCloud(std::vector<Point3Normal> &points_with_normal,
                         std::vector<Color> &colors);

    protected:
      Options options_;
      std::vector<ProjectionMatrix> projection_matrices_;
    };
  }
}

#endif DENSEPOINTS_PMVS_PMVS
