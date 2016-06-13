#include "patch_organizer.h"
#include "easylogging/easylogging.h"

namespace DensePoints {
namespace PMVS {

PatchGrid::PatchGrid(size_t width, size_t height, size_t max_patches_per_cell)
{
  grid_ = std::vector<PatchGridCell>(width * height);
  width_ = width;
  height_ = height;
  max_patches_per_cell_ = max_patches_per_cell;
}

bool PatchGrid::TryInsert(size_t row, size_t column, const Patch &patch)
{
  // Check if it is inside bounds
  if (column >= 0 && column < width_ && row >= 0 && row < height_) {
    // Check if there is space
    PatchGridCell &patch_grid_cell = grid_[row * height_ + column];
    if (patch_grid_cell.size() < max_patches_per_cell_) {
      // Insert a pointer to the patch
      patch_grid_cell.push_back(&patch);
      return true;
    }
  } else {
    LOG(INFO) << "Rejecting patch out-of-bounds at " << row << ", " << column;
  }
  return false;
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
  PatchCells patch_cells;
  for (const auto view_id : patch.GetTrullyVisibleImages()) {
    Vector2 point = (*views_)[view_id].ProjectPoint(patch.GetPosition());
    size_t row = static_cast<size_t>(point[1] / options_.grid_scale);
    size_t col = static_cast<size_t>(point[0] / options_.grid_scale);

    if (patch_set_[view_id].TryInsert(row, col, patch)) {
      patch_cells[view_id] = std::pair<size_t, size_t>(row, col);
    }
  }
  // Automatically tell the original patch where it will leave in the grid,
  // and also add the patch to the vector
  if (patch_cells.size() > 0) {
    Patch new_patch = patch;
    new_patch.SetPatchCells(patch_cells);
    patches_.push_back(new_patch);
  }
  return patch_cells;
}

// Make sure both patches and and the PatchCells are in sync, that is,
// that patches have the correct views where they are in the grid, but,
// that it is also store in the patch data
void PatchOrganizer::SetSeeds(const std::vector<Patch> &seeds)
{
  for (const Patch &patch : seeds) {
    TryInsert(patch);
  }
}

std::vector<Patch*> PatchOrganizer::GetPatchesPointers()
{
  std::vector<Patch*> patch_pointers;
  for (auto &patch : patches_) {
    patch_pointers.push_back(&patch);
  }
  return patch_pointers;
}

}
}
