#ifndef DENSEPOINTS_PMVS_PATCH
#define DENSEPOINTS_PMVS_PATCH

#include <vector>
#include "core/types.h"

namespace DensePoints {
  namespace PMVS {

    typedef std::vector<size_t> ImagesIndices;

    class Patch {
    public:
      bool IsAcceptable();
      float NCCScore(Patch &comparison_patch);
      const PointCloudXYZRGBNormal GetPoint();
    private:
      // PointXYZ and Normal data is stored in PointXYZRGBNormal
      size_t pointcloud_index_;

      // Additional information of the patch
      float score_;
      size_t reference_image_;
      ImagesIndices visible_images_;
      ImagesIndices candidate_images_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_PATCH
