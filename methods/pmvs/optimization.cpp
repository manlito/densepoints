#include "optimization.h"
#include "core/error_measurements.h"
#include "easylogging/easylogging.h"
#include <opencv2/imgproc.hpp>

using namespace DensePoints;
using namespace DensePoints::PMVS;

void Optimization::GetProjectedTextures(std::vector<cv::Mat> &textures)
{
  // Project unitary vectors using patch parameters into ref image
  // Unitary vectors should use corresponding roll and pitch

  // Get a vector in the same direction as the X direction of the image
  const View &reference_view = views_[patch_.GetReferenceImage()];
  Vector3 patch_normal = patch_.GetNormal();
  Vector3 reference_view_x_axis = reference_view.GetXAxis();
  Vector3 reference_view_y_axis = patch_normal.cross(reference_view_x_axis);

  // Take the largest size, as projected in the image
  Vector2 projected_center, projected_x, projected_y;
  projected_center = reference_view.ProjectPoint(patch_.GetPosition());
  projected_x = reference_view.ProjectPoint(patch_.GetPosition() + reference_view_x_axis);
  double dx = (projected_x - projected_center).norm();

  LOG_IF(dx == 0, FATAL) << "Projection of unitary vector on x-axis retuned 0 length vector";

  // Compute a scale, using projection on image x-axis
  double scale = (cell_size_ / 2 ) / dx;

  // Project all patches with the computed scale
  for (const size_t view_index : patch_.GetTrullyVisibleImages())
  {
    cv::Mat homography;
    cv::Rect2i roi;
    bool valid_texture = patch_.ComputePatchToViewHomography(views_[view_index],
                                                             cell_size_,
                                                             scale * reference_view_x_axis,
                                                             scale * reference_view_y_axis,
                                                             homography,
                                                             roi);
    // Insert an empty image for texture where at least one corner
    // is outside the image
    if (!valid_texture) {
      textures.push_back(cv::Mat());
      continue;
    }

    // Warp the homography to extract the patch
    cv::Mat texture;
    cv::Mat &image = views_[view_index].GetImage();
    cv::warpPerspective(image(roi), texture, homography, cv::Size(cell_size_, cell_size_), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    textures.push_back(texture);
  }
}

void Optimization::ParametrizePatch(double &depth, double &roll, double &pitch)
{
  // Depth is the length of the vector from patch to reference image center
  depth = (patch_.GetPosition() - views_[patch_.GetReferenceImage()].GetCameraCenter()).norm();
}

void Optimization::FilterByErrorMeasurement()
{
  // Textures are obtained in the same order as TrullyVisible array
  std::vector<cv::Mat> textures;
  GetProjectedTextures(textures);
  // Store the Score
  std::vector<double> scores;
  std::stringstream scores_print;
  for (size_t texture_index = 0; texture_index < textures.size(); ++texture_index) {
    if (texture_index > 0) {
      double score = NCCScore(textures[0], textures[texture_index]);
      scores.push_back(score);
      scores_print << score << " ";
    } else {
      scores.push_back(0);
    }
  }
  LOG(INFO) << "Score: " << scores_print.str();
}
