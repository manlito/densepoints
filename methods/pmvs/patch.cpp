#include "patch.h"
#include <math.h>

using namespace DensePoints;
using namespace DensePoints::PMVS;

void Patch::InitRelatedImages(const std::vector<View> &views,
                              double trully_visible_threshold,
                              double potentially_visible_threshold)
{
  const Vector3 normal = GetNormal();
  const Vector3 position = GetPosition();
  visible_images_.clear();
  candidate_images_.clear();

  // Project the patch to each image
  for (size_t view_index = 0; view_index < views.size(); ++view_index) {
    if (view_index != reference_image_) {
      const View &view = views[view_index];

      // Check the point is actually inside the other image
      if (!view.IsPointInside(position)) {
        continue;
      }

      // Check the angle between the normal and the optical ray
      // from candidate image
      Vector3 patch_to_view = position - view.GetCameraCenter();
      double angle = std::acos(normal.dot(patch_to_view) / patch_to_view.norm());
      if (angle < trully_visible_threshold) {
        visible_images_.push_back(view_index);
      } else if (angle < potentially_visible_threshold) {
        candidate_images_.push_back(view_index);
      }
    }
  }
}
