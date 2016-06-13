#include <queue>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "expand.h"
#include "io/file_utils.h"
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
#endif

  ExpandPatches();

}

void Expand::ExpandPatches()
{
  // Comparison function
  auto compare_patches = [](Patch* left, Patch* right) {
    return left->GetTrullyVisibleImages().size() < right->GetTrullyVisibleImages().size();
  };
  // Create a list of patches that need expansion
  std::priority_queue<Patch*, std::vector<Patch*>, decltype(compare_patches)>
      patches_to_expand(compare_patches, patch_organizer_->GetPatchesPointers());

  // A omp parallel will go here
  {
    while (true)
    {
      Patch* patch = nullptr;
      // Critical here
      {
        if (patches_to_expand.size() > 0) {
          patch = patches_to_expand.top();
          patches_to_expand.pop();
        }
      }
      if (patch != nullptr) {
        LOG(INFO) << "Expading patch with coordinates: " << patch->GetPosition().transpose();
        ExpandPatch(patch);
      } else {
        // Exit as no more patches to expand are left
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
  patch->GetProjectedXYAxisAndScale(reference_view,x_axis, y_axis, dx);

  double scale = 1 / dx;

  const std::vector<Vector3> expansion_directions {
    Vector3(1, 0, 0), Vector3(-1, 0, 0), Vector3(0, 1, 0), Vector3(0, -1, 0)
  };

  // Each direction to expand
  for (const Vector3 direction : expansion_directions) {
    Vector3 position = patch->GetPosition() + scale * direction;
    LOG(INFO) << "Exp: " << position.transpose();
  }

  // Project the center and the right axis
  Patches patches;
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
