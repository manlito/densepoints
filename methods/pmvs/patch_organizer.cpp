#include "patch_organizer.h"

namespace DensePoints {
namespace PMVS {

PatchGrid::PatchGrid(size_t width, size_t height, size_t max_patches_per_cell)
{
  grid_ = std::vector<PatchGridCell>(width * height);
}

void PatchOrganizer::AllocateViews()
{
  for (View view : *views_) {
    size_t grid_width = view.GetImage().cols / options_.grid_scale;
    size_t grid_height = view.GetImage().rows / options_.grid_scale;
    PatchGrid patch_grid(grid_width, grid_height, options_.max_patches_per_cell);
    patch_set_.push_back(patch_grid);
  }
}

PatchCells PatchOrganizer::TryInsert(const Patch &patch)
{
  for (const auto view_id : patch.GetTrullyVisibleImages()) {
    Vector2 point = (*views_)[view_id].ProjectPoint(patch.GetPosition());
    size_t row = static_cast<size_t>(point[1] / options_.grid_scale);
    size_t col = static_cast<size_t>(point[0] / options_.grid_scale);

    PatchCells patch_cells;
    if (patch_set_[view_id].TryInsert(row, col, patch)) {
      patch_cells[view_id] = std::pair<size_t, size_t>(row, col);
    }
  }
}

void PatchOrganizer::SetSeeds(const std::vector<Patch> &seeds)
{
  for (const Patch &patch : seeds) {
    TryInsert(patch);
  }
}

}
}
