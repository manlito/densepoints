#ifndef DENSEPOINTS_PMVS_PATCH
#define DENSEPOINTS_PMVS_PATCH

#include <vector>
#include "core/types.h"

namespace DensePoints {
  namespace PMVS {

    typedef std::vector<size_t> ImagesIndices;

    class Patch {
    public:

      // Generates a homography than can be used to warp path's projection
      // to a given image
      bool ComputePatchToViewHomography(View &view,
                                        size_t cell_size,
                                        const Vector3 &x_axis,
                                        const Vector3 &y_axis,
                                        cv::Mat &homography,
                                        cv::Rect2i &roi);

      void SetReferenceImage(size_t image_index) {
        reference_image_ = image_index;
      }
      size_t GetReferenceImage() const { return reference_image_; }

      void SetNormal(Vector3 normal) {
        point_.normal_x = normal[0];
        point_.normal_y = normal[1];
        point_.normal_z = normal[2];
      }
      Vector3 GetNormal() const {
        return Vector3(point_.normal_x, point_.normal_y, point_.normal_z);
      }
      void SetPosition(Vector3 position) {
        point_.x = position[0];
        point_.y = position[1];
        point_.z = position[2];
      }
      Vector3 GetPosition() const {
        return Vector3(point_.x, point_.y, point_.z);
      }
      void InitRelatedImages(const std::vector<View> &views,
                             double trully_visible_threshold = 0.78, // Pi/4
                             double potentially_visible_threshold = 1.04); // Pi/3
      const ImagesIndices& GetTrullyVisibleImages() const {
        return visible_images_;
      }
      const ImagesIndices& GetPotentiallyVisibleImages() const {
        return candidate_images_;
      }

      float ComputeScore();
      const PointXYZRGBNormal GetPoint() const { return point_; }
    private:
      // For patch optimization, we keep a copy here
      PointXYZRGBNormal point_;

      // PointXYZ and Normal data is stored in PointXYZRGBNormal
      size_t pointcloud_index_;

      // Additional information of the patch
      double score_;
      size_t reference_image_;
      ImagesIndices visible_images_;
      ImagesIndices candidate_images_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_PATCH
