#include "patch.h"
#include <math.h>
#include <opencv2/calib3d.hpp>

using namespace DensePoints;
using namespace DensePoints::PMVS;

void Patch::InitRelatedImages(const Views views,
                              double trully_visible_threshold,
                              double potentially_visible_threshold)
{
  const Vector3 normal = GetNormal();
  const Vector3 position = GetPosition();
  visible_images_.clear();
  candidate_images_.clear();

  // Project the patch to each image
  for (size_t view_index = 0; view_index < views->size(); ++view_index) {
    if (view_index != reference_image_) {
      const View &view = (*views)[view_index];

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

void Patch::RemoveTrullyVisibleImage(size_t index)
{
  visible_images_.erase(visible_images_.begin() + index);
}

bool Patch::ComputePatchToViewHomography(View &view,
                                         size_t cell_size,
                                         const Vector3 &x_axis,
                                         const Vector3 &y_axis,
                                         cv::Mat &homography,
                                         cv::Rect2i &roi)
{
  // Get the points in the local world
  std::vector<Vector3> patch_corners;
  patch_corners.push_back(GetPosition() - x_axis - y_axis);
  patch_corners.push_back(GetPosition() + x_axis - y_axis);
  patch_corners.push_back(GetPosition() + x_axis + y_axis);
  patch_corners.push_back(GetPosition() - x_axis + y_axis);

  // Find the enclosing square for more efficient homography warping
  cv::Point2i top_left(view.GetImage().cols, view.GetImage().rows), bottom_right(0, 0);
  std::vector<cv::Point2f> points_projected;
  for (size_t i = 0; i < 4; ++i) {
    // Return false when any of the corners of the patch are outside the image
    if (!view.IsPointInside(patch_corners[i])) {
      return false;
    }
    const Vector2 projected_point = view.ProjectPoint(patch_corners[i]);
    points_projected.push_back(cv::Point2f(projected_point[0], projected_point[1]));

    // Find the minimum enclosing rectangle
    top_left.x = std::min(top_left.x, static_cast<int>(std::ceil(projected_point[0])));
    top_left.y = std::min(top_left.y, static_cast<int>(std::ceil(projected_point[1])));
    bottom_right.x = std::max(bottom_right.x, static_cast<int>(std::floor(projected_point[0])));
    bottom_right.y = std::max(bottom_right.y, static_cast<int>(std::floor(projected_point[1])));
  }

  // Apply ROI
  roi.x = top_left.x;
  roi.y = top_left.y;
  roi.width = bottom_right.x - top_left.x;
  roi.height = bottom_right.y - top_left.y;
  for (size_t i = 0; i < 4; ++i) {
    points_projected[i].x -= roi.x;
    points_projected[i].y -= roi.y;
  }

  // Corners of the cell size rectangle
  std::vector<cv::Point2f> points_cell;
  points_cell.push_back(cv::Point2f(0, 0));
  points_cell.push_back(cv::Point2f(cell_size, 0));
  points_cell.push_back(cv::Point2f(cell_size, cell_size));
  points_cell.push_back(cv::Point2f(0, cell_size));

  // TODO: Check cost of LM done by this method
  homography = cv::findHomography(points_projected, points_cell, 0);

  return true;
}
