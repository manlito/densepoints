#include <queue>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "expand.h"
#include "optimization_opencv.h"
#include "io/file_utils.h"
#include "utils.h"
#include "easylogging/easylogging.h"

namespace DensePoints {
namespace PMVS {

void Expand::SetSeeds(const Patches seeds)
{
  patch_organizer_.reset();
  patch_organizer_ = std::unique_ptr<PatchOrganizer>(new PatchOrganizer(views_));
  patch_organizer_->AllocateViews();
  patch_organizer_->SetSeeds(seeds);

#ifdef DEBUG_PMVS_EXPANSION
  PrintPatchGrids("initial");
  PrintCloud(patch_organizer_->GetPatches(), "points", "before_expansion");
#endif

  ExpandPatches();

#ifdef DEBUG_PMVS_EXPANSION
  PrintCloud(patch_organizer_->GetPatches(), "points", "after_expansion");
#endif

}

void Expand::ExpandPatches()
{
//  // Comparison function
//  auto compare_patches = [](Patch* left, Patch* right) {
//    return left->GetTrullyVisibleImages().size() < right->GetTrullyVisibleImages().size();
//  };
//  // Create a list of patches that need expansion
//  std::priority_queue<Patch*, std::vector<Patch*>, decltype(compare_patches)>
//      patches_to_expand(compare_patches, patch_organizer_->GetPatchesPointers());
  // Comparison function
  // Create a list of patches that need expansion
  std::queue<Patch*> patches_to_expand;
  for (const auto patch_pointer : patch_organizer_->GetPatchesPointers()) {
    patches_to_expand.push(patch_pointer);
  }
//  patches_to_expand.push( patch_organizer_->GetPatchesPointers()[0] );

  size_t patch_count = 0;
#pragma omp parallel
  {
    while (true)
    {
      Patch* patch = nullptr;

#pragma omp critical
      {
        // Grab a patch
        if (patches_to_expand.size() > 0) {
          patch = patches_to_expand.front();
          patches_to_expand.pop();
        }
      }

      if (patch != nullptr) {
        // Only try to expand if has enough visible images
        if (patch->GetTrullyVisibleImages().size() >= 2) {
          Patches patches = ExpandPatch(patch);
          for (const Patch &new_patch : patches) {
#pragma omp critical
            {
              Patch *inserted_patch = patch_organizer_->TryInsert(new_patch);
              if (inserted_patch != nullptr) {
                patches_to_expand.push(inserted_patch);
              }
            }
          }
        } else {
          //LOG(INFO) << "Found patch " << patch->GetPosition().transpose() <<" with " << patch->GetTrullyVisibleImages().size() << " views";
        }
      } else {
        // Exit as no more patches to expand are left
        break;
      }

#pragma omp critical
      {
        patch_count++;
        if (patch_count % 1000 == 0) {
          LOG(INFO) << patch_count << " patches so far";
        }
      }
      if (patch_count >= 5e5) {
        break;
      }

    }
  }
}

Patches Expand::ExpandPatch(Patch *const patch)
{
  // Expand using patch normal and the x-axis of the image.
  View &reference_view = (*views_)[patch->GetReferenceImage()];

  Vector3 x_axis, y_axis;
  double dx;
  patch->GetProjectedXYAxisAndScale(reference_view, x_axis, y_axis, dx);

  double scale = patch_organizer_->GetOptions().grid_scale / dx;

  const std::vector<Vector3> expansion_directions {
    x_axis, -x_axis, y_axis, -y_axis
  };
  // Project the center and the right axis
  Patches patches;

  // Each direction to expand
  // LOG(INFO) << "P: " << reference_view.ProjectPoint(patch->GetPosition()).transpose();
  for (const Vector3 direction : expansion_directions) {
    Vector3 position = patch->GetPosition() + scale * direction;
    Vector2 projected_point = reference_view.ProjectPoint(position);
    // LOG(INFO) << "Exp: " << position.transpose();
    Patch new_patch = *patch;
    new_patch.SetPosition(position);

    OptimizationOpenCV optimizer(new_patch, views_, options_.cell_size);
    if (optimizer.Optimize()) {
      // Recompute visible images
      new_patch.InitRelatedImages(views_);
      if (optimizer.FilterByErrorMeasurement()) {
        // Accept if they exceed the threhold
        patches.push_back(new_patch);
      }
    } else {
      LOG(INFO) << "Failed expansion";
    }
  }

  return patches;
}

void Expand::PrintPatchGrids(const std::string folder_name)
{
  const std::string output_folder = IO::GetFolder({ DEBUG_OUTPUT_PATH, "patch_organizer", "expansion", folder_name});
  size_t max_patches_per_cell = patch_organizer_->GetOptions().max_patches_per_cell;
  for (size_t patch_grid_index = 0;
       patch_grid_index < patch_organizer_->Size();
       ++patch_grid_index) {
    PatchGrid &patch_grid = patch_organizer_->At(patch_grid_index);
    cv::Mat image(patch_grid.Height(), patch_grid.Width(), CV_8UC1);
    image.setTo(0);

    auto image_iterator = image.begin<unsigned char>();
    for (PatchGridCell &patch_grid_cell : patch_grid) {
      *image_iterator = static_cast<unsigned char>(
                          255 * (static_cast<double>(patch_grid_cell.size())
                          / static_cast<double>(max_patches_per_cell))
                       );
      ++image_iterator;
    }
    cv::imwrite(stlplus::create_filespec(output_folder,
                                         std::string("view_") + std::to_string(patch_grid_index),
                                         ".jpg"), image);
  }
}


}
}
