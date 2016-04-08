#include "optimization.h"
#include "easylogging/easylogging.h"

using namespace DensePoints;
using namespace DensePoints::PMVS;

bool Optimization::Optimize()
{
  // Grab textures

  // Parametrize

  // Run the actual optimization

  // Unparametrize

  return true;
}

void Optimization::GetProjectedTextures(std::vector<cv::Mat> &textures) const
{
  // Project unitary vectors using patch parameters into ref image
  // - Unitary vectors should use corresponding roll and pitch

  // Get a vector in the same direction as the X direction of the image
  const View &reference_view = views_[patch_.GetReferenceImage()];
  Vector3 patch_normal = patch_.GetNormal();
  Vector3 reference_view_x_axis = reference_view.GetXAxis();
  Vector3 reference_view_y_axis = reference_view_x_axis.cross(patch_normal);

  // Take the largest size, as projected in the image
  Vector2 projected_center, projected_x, projected_y;
  projected_center = reference_view.ProjectPoint(patch_.GetPosition());
  projected_x = reference_view.ProjectPoint(patch_.GetPosition() + reference_view_x_axis);
  projected_y = reference_view.ProjectPoint(patch_.GetPosition() + reference_view_y_axis);
  LOG(INFO) << projected_center[0] << ":" << projected_center[1]
            << " / " <<  projected_x[0] << ":" << projected_x[1]
            << " / " <<  projected_y[0] << ":" << projected_y[1]
            << ", dx: " << (projected_x - projected_center).norm()
            << ", dy: " << (projected_y - projected_center).norm();

  //  - Compute a scale, taking the patch width and the projected largest size
  //  - Multiply unitary vectors around patch by this scale
  //  - Project all patches with the computed scale
  //  -- For each patch:
  //  -- Extract the minimum rectangle containing all four corners
  //  -- Compute a homography for from unitary square patch_size x patch_size
  //     and the four corners, translated by the top left corner
  //  -- Warp the ROI using the computed homography
  //  -- Resize the warped rectangle
}
