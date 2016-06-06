#include "optimization.h"
#include "core/error_measurements.h"
#include "easylogging/easylogging.h"
#include <opencv2/imgproc.hpp>

using namespace DensePoints;
using namespace DensePoints::PMVS;

void Optimization::GetProjectedTextures(std::vector<cv::Mat> &textures)
{
  GetProjectedTextures(patch_.GetNormal(), patch_.GetPosition(), textures);
}

void Optimization::GetProjectedTextures(const Vector3 normal, const Vector3 position,
                                        std::vector<cv::Mat> &textures)
{
  // Project unitary vectors using patch parameters into ref image
  // Unitary vectors should use corresponding roll and pitch

  // Get a vector in the same direction as the X direction of the image
  const View &reference_view = views_[patch_.GetReferenceImage()];
  Vector3 reference_view_x_axis = reference_view.GetXAxis();
  Vector3 reference_view_y_axis = normal.cross(reference_view_x_axis);

  // Take the largest size, as projected in the image
  Vector2 projected_center, projected_x, projected_y;
  projected_center = reference_view.ProjectPoint(position);
  projected_x = reference_view.ProjectPoint(position + reference_view_x_axis);
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
    if (!valid_texture || roi.width <= 0 || roi.height <= 0) {
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
  ParametrizePatch(patch_.GetNormal(), patch_.GetPosition(), depth, roll, pitch);
}

void Optimization::ParametrizePatch(const Vector3 normal, const Vector3 position,
                                    double &depth, double &roll, double &pitch)
{
  // Depth is the length of the vector from patch to reference image center
  depth = (position - views_[patch_.GetReferenceImage()].GetCameraCenter()).norm();
  // As for angles, we know the patch x-axis is aligned with image x-axis.
  // Let's build a rotatio matrix, then a rotation matrix to angle conversion
  Vector3 x_axis = views_[patch_.GetReferenceImage()].GetXAxis().normalized();
  Vector3 y_axis = normal.cross(x_axis);
  Vector3 z_axis = x_axis.cross(y_axis);

  roll = std::atan2(z_axis(1), z_axis(2));
  pitch = std::atan2(-z_axis(0), std::sqrt(std::pow(z_axis(1), 2) + std::pow(z_axis(2), 2)));
}

void Optimization::UnparametrizePatch(double depth, double roll, double pitch,
                                      Vector3 &normal, Vector3 &position)
{
  const Vector3 &camera_center = views_[patch_.GetReferenceImage()].GetCameraCenter();
  const Vector3 &current_position = patch_.GetPosition();
  position = camera_center + (1 + depth) * (current_position - camera_center);
  // This method assumes given angles are relative, so that we onlu
  // need to compute a rotation after the initial patch angle
  double ca = std::cos(roll);
  double sa = std::sin(roll);
  double cb = std::cos(pitch);
  double sb = std::sin(pitch);
  Matrix3 rotation;
  rotation.row(0) <<      cb,   0,     -sb;
  rotation.row(1) << sa * sb,  ca, cb * sa ;
  rotation.row(2) << ca * sb, -sa, ca * cb;
//  rotation.row(0) <<      cb,   0,     -sb;
//  rotation.row(1) << sa * sb,  ca, cb * sa ;
//  rotation.row(2) << ca * sb, -sa, ca * cb;
  // Compositional update
  normal = rotation * patch_.GetNormal();
}

bool Optimization::FilterByErrorMeasurement()
{
  // Textures are obtained in the same order as TrullyVisible array
  std::vector<cv::Mat> textures;
  GetProjectedTextures(textures);
  // Store the Score
  std::vector<double> scores;
  for (size_t texture_index = 0; texture_index < textures.size(); ++texture_index) {
    if (texture_index > 0) {
      double score = NCCScore(textures[0], textures[texture_index]);
      scores.push_back(score);
    }
  }

  // Remove cases with empty scores
  if (scores.size() == 0) {
    return false;
  }

  // Remove TrullyVisible that have a low score
  size_t removed_images = 0;
  for (size_t score_index = 0; score_index < scores.size(); ++score_index) {
    if (scores[score_index] < score_threshold_) {
      patch_.RemoveTrullyVisibleImage(score_index - removed_images);
      ++removed_images;
    }
  }

  // Drop the patch if it finally has less than minimum_visible_image images
  if (patch_.GetTrullyVisibleImages().size() < minimum_visible_image_) {
    return false;
  }

  return true;
}
