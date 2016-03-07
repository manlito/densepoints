#ifndef DENSEPOINTS_PMVS_PATCH
#define DENSEPOINTS_PMVS_PATCH

#include <vector>
#include "core/types.h"

namespace DensePoints {
  namespace PMVS {

    typedef std::vector<size_t> ImagesIndices;

    class Patch {
    public:
      void SetReferenceImage(size_t image_index) {
        reference_image_ = image_index;
      }
      void SetNormal(Vector3 normal) {
        point_.normal_x = normal[0];
        point_.normal_y = normal[1];
        point_.normal_z = normal[2];
      }
      void SetPosition(Vector3 position) {
        point_.x = position[0];
        point_.y = position[1];
        point_.z = position[2];
      }
      bool IsAcceptable();
      float NCCScore(Patch &comparison_patch);
      const PointXYZRGBNormal GetPoint() const { return point_; }
    private:
      // For patch optimization, we keep a copy here
      PointXYZRGBNormal point_;

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
