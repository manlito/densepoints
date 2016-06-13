#ifndef DENSEPOINTS_PMVS_PATCH
#define DENSEPOINTS_PMVS_PATCH

#include <vector>
#include <map>
#include "core/types.h"

namespace DensePoints {
namespace PMVS {

// A set of images indices, or view_ids
typedef std::vector<size_t> ImagesIndices;

// Note that a patch may not be present in all of its visible_images_,
// because there is a limitation on the number of patches per cell
// map <view_id, pair <row, column>>
typedef std::map<size_t, std::pair<size_t, size_t>> PatchCells;

enum class ErrorMeasurement { NCC, SSD, SAD };

class Patch {
public:

  // Generates a homography than can be used to warp patch's projection
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
  void InitRelatedImages(const Views views,
                         double trully_visible_threshold = 0.78, // Pi/4
                         double potentially_visible_threshold = 1.04); // Pi/3
  const ImagesIndices& GetTrullyVisibleImages() const {
    return visible_images_;
  }
  const ImagesIndices& GetPotentiallyVisibleImages() const {
    return candidate_images_;
  }
  void GetProjectedXYAxisAndScale(const View &view,
                                  Vector3 &x_axis, Vector3 &y_axis,
                                  double &scale);
  void GetProjectedXYAxisAndScale(const View &view,
                                  Vector3 normal, Vector3 position,
                                  Vector3 &x_axis, Vector3 &y_axis,
                                  double &scale);
  void RemoveTrullyVisibleImage(size_t index);

  // Sets the map that indicates where this patch is
  // using a spot in the patch organizer
  void SetPatchCells(const PatchCells &patch_cells) {
    patch_cells_ = patch_cells;
  }

  // Patch projection and textures
  float ComputeScore(Patch &patch);
  const PointXYZRGBNormal GetPoint() const { return point_; }
private:
  // For patch optimization, we keep a copy here
  PointXYZRGBNormal point_;

  // PointXYZ and Normal data is stored in PointXYZRGBNormal
  size_t pointcloud_index_;

  // Additional information of the patch
  ErrorMeasurement error_measurement_ = ErrorMeasurement::SSD;
  size_t reference_image_;
  ImagesIndices visible_images_;
  ImagesIndices candidate_images_;

  // A quick accesor to the cells in the patch organizer
  // where this patch is available. Used during expansion
  PatchCells patch_cells_;
};

typedef std::vector<Patch> Patches;

}
}

#endif // DENSEPOINTS_PMVS_PATCH
