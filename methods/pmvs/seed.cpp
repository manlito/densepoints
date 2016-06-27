#include <algorithm>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/features2d.hpp>
#include "easylogging/easylogging.h"
#include "seed.h"
#include "optimization_opencv.h"
#include "utils.h"
#include "core/grid.h"
#include "geometry/fundamental_matrix.h"
#include "geometry/triangulation.h"
#include "rplycpp/src/rplycpp.hpp"
#include "io/file_utils.h"

using namespace DensePoints;
using namespace DensePoints::PMVS;

void Seed::ConvertSeedsToPatches()
{
  CreatePatchesFromPoints();
  OptimizeAndRefinePatches();
}

void Seed::CreatePatchesFromPoints()
{
#pragma omp parallel for
  for (size_t point_index = 0; point_index < points_.size(); ++point_index) {
    // Look for a reference image by distance to the camera center
    // to the optical center
    const Vector3 point = points_[point_index];
    double min_distance = (point - (*views_)[0].GetCameraCenter()).norm();
    size_t min_index = 0;
    for (size_t camera_index = 1; camera_index < views_->size(); ++camera_index) {
      double distance = (point - (*views_)[camera_index].GetCameraCenter()).norm();
      if (distance < min_distance) {
        min_index = camera_index;
        min_distance = distance;
      }
    }
    Vector3 patch_to_center = point - (*views_)[min_index].GetCameraCenter();
    Vector3 normal = patch_to_center / patch_to_center.norm();
    // Init patch
    Patch patch;
    patch.SetReferenceImage(min_index);
    patch.SetPosition(point);
    patch.SetNormal(normal);
    patch.InitRelatedImages(views_);
#pragma omp critical
    {
      patches_.push_back(patch);
    }
  }

#ifdef DEBUG_PMVS_SEEDS
  // Debug patches
  rplycpp::PLYWriter writer;
  writer.Open(stlplus::create_filespec(IO::GetFolder({ DEBUG_OUTPUT_PATH, "points" }),
                                       "patches_seed", "ply"));
  std::vector<rplycpp::PLYProperty> properties;
  rplycpp::PLYProperty property;
  property.type = rplycpp::PLYDataType::PLY_FLOAT;
  property.name = "x";
  properties.push_back(property);
  property.name = "y";
  properties.push_back(property);
  property.name = "z";
  properties.push_back(property);
  property.name = "nx";
  properties.push_back(property);
  property.name = "ny";
  properties.push_back(property);
  property.name = "nz";
  properties.push_back(property);
  writer.AddElement("vertex", properties, points_.size());

  // Add the data IN THE SAME ORDER!
  for (const Patch &patch : patches_) {
    const PointXYZRGBNormal point = patch.GetPoint();
    writer.AddRow(std::vector<double> { point.x, point.y, point.z,
                                        point.normal_x, point.normal_y, point.normal_z});
  }
  writer.Close();
#endif
}

void Seed::OptimizeAndRefinePatches()
{
#ifdef DEBUG_PMVS_OPTIMIZATION
  PrintTextures("initial_projected");
#endif

  // Refine patches
  FilterPatches();
#ifdef DEBUG_PMVS_OPTIMIZATION
  PrintTextures("initial_filtered");
  PrintCloud(patches_, "points", "initial_filtered");
#endif

  // Patch optimization
  OptimizePatches();
#ifdef DEBUG_PMVS_OPTIMIZATION
     PrintTextures("initial_optimized");
     PrintCloud(patches_, "points", "initial_optimized");
#endif

}

void Seed::FilterPatches()
{
  // Filter by NCC score
  std::vector<size_t> patches_to_remove;
  {
    for (size_t patch_index = 0; patch_index < patches_.size(); ++patch_index) {
      Patch &patch = patches_[patch_index];
      OptimizationOpenCV optimizer(patch, views_, options_.cell_size);
      if (!optimizer.FilterByErrorMeasurement()) {
        // Mark the patch for removal
        patches_to_remove.push_back(patch_index);
      }
    }
  }
  // Prune patches
  RemovePatches(patches_to_remove);
}

void Seed::OptimizePatches()
{
  std::vector<size_t> patches_to_remove;

  // Optimize patches
  for (size_t patch_index = 0; patch_index < patches_.size(); ++patch_index) {
    Patch &patch = patches_[patch_index];
    OptimizationOpenCV optimizer(patch, views_, options_.cell_size);
    if (!optimizer.Optimize()) {
      // Mark the patch for removal
      patches_to_remove.push_back(patch_index);
    }
  }

  // Prune patches
  RemovePatches(patches_to_remove);
}

void Seed::RemovePatches(const std::vector<size_t> &patch_indices)
{
  LOG(INFO) << patch_indices.size() << " patches need to be removed after filter by error measurement";
  size_t remove_offset = 0;
  for (size_t index_to_remove : patch_indices) {
    auto patch_iterator = patches_.begin();
    std::advance(patch_iterator, index_to_remove - remove_offset);
    patches_.erase(patch_iterator);
    ++remove_offset;
  }
}

void Seed::PrintPatches(const std::string folder_name)
{
  const std::string output_folder = IO::GetFolder({ DEBUG_OUTPUT_PATH, "patches", "seeds", folder_name});
  const size_t patch_radius = seed_options_.patch_size / 2;
  for (size_t patch_index = 0; patch_index < patches_.size(); ++patch_index) {
    // For each patch, get its related images
    Patch &patch = patches_[patch_index];
    const ImagesIndices& visible_images = patch.GetTrullyVisibleImages();
    const ImagesIndices& candidate_images = patch.GetPotentiallyVisibleImages();

    // Some process should verify images with boundaries outside of the image
    for (const size_t view_index : visible_images) {
      const cv::Mat &image = (*views_)[view_index].GetImage();
      const Vector2 projected_point = (*views_)[view_index].ProjectPoint(patch.GetPosition());
      if (projected_point[0] > patch_radius && projected_point[0] < image.cols - patch_radius &&
          projected_point[1] > patch_radius && projected_point[1] < image.rows - patch_radius) {
        cv::imwrite(stlplus::create_filespec(output_folder, std::string("patch_") + std::to_string(patch_index) + "_v_" + std::to_string(view_index), "jpg"),
                    image(cv::Rect(projected_point[0] - patch_radius, projected_point[1] - patch_radius, seed_options_.patch_size, seed_options_.patch_size)));
      }

    }
  }
}

void Seed::PrintTextures(const std::string folder_name)
{
  const std::string output_folder = IO::GetFolder({ DEBUG_OUTPUT_PATH, "patches", "seeds", folder_name});
  if (!stlplus::folder_exists(output_folder)) {
    stlplus::folder_create(output_folder);
  }
  const size_t patch_radius = seed_options_.patch_size / 2;
  for (size_t patch_index = 0; patch_index < patches_.size(); ++patch_index) {
    Patch &patch = patches_[patch_index];

    // Texture grabbing is part of optimization class...
    // perhaps moving to separte class?
    OptimizationOpenCV optimizer(patch, views_, options_.cell_size);

    // Debug texture grabbing
    std::vector<cv::Mat> textures;
    std::vector<size_t> invalid_views;
    optimizer.GetProjectedTextures(textures);
    size_t view_index = 0;
    for (const cv::Mat &texture : textures) {
      if (!texture.empty()) {
        cv::imwrite(stlplus::create_filespec(
                      output_folder,
                      std::string("tex_") + std::to_string(patch_index) + "_" + std::to_string(patch.GetTrullyVisibleImages()[view_index]),
                      "jpg"),
                    texture);
      }
      ++view_index;
    }
  }
}
