#ifndef DENSEPOINTS_PMVS_PATCH_ORGANIZER
#define DENSEPOINTS_PMVS_PATCH_ORGANIZER

#include <vector>
#include <map>
#include "patch.h"

namespace DensePoints {
namespace PMVS {

// Make sure you delete this points when removing patch
typedef std::vector<Patch const*> PatchGridCell;

class PatchGrid {
public:
  PatchGrid(size_t width, size_t height, size_t max_patches_per_cell);
  const PatchGridCell& GetAt(size_t row, size_t column) const;
  bool TryInsert(size_t row, size_t column, const Patch &patch);
  size_t Width() const { return width_; }
  size_t Height() const { return height_; }

  // Iterators
  PatchGridCell* begin() {
    if (grid_.size() > 0)
      return &grid_.front();
    return nullptr;
  }
  PatchGridCell* end() {
    if (grid_.size() > 0)
      return &grid_.back();
    return nullptr;
  }
private:
  size_t width_;
  size_t height_;
  size_t max_patches_per_cell_;
  std::vector<PatchGridCell> grid_;
};

struct PatchOrganizerOptions {
  size_t max_patches_per_cell;
  size_t grid_scale;
  PatchOrganizerOptions(size_t max_patches_per_cell = 1,
                        size_t grid_scale = 8)
    : max_patches_per_cell(max_patches_per_cell),
      grid_scale(grid_scale) { }
};

typedef std::vector<PatchGrid> PatchSet;
class PatchOrganizer {
public:
  PatchOrganizer(Views views,
                 PatchOrganizerOptions options = PatchOrganizerOptions())
    : views_(views),
      options_(options) { patches_.reserve(1e6); }
  void AllocateViews();

  PatchOrganizerOptions GetOptions() { return options_; }
  void SetOptions(const PatchOrganizerOptions options) {
    options_ = options;
  }
  // Getters and setters
  size_t Size() const { return patch_set_.size(); }
  PatchGrid& At(size_t index) { return patch_set_[index]; }
  std::vector<Patch*> GetPatchesPointers();
  Patches& GetPatches() { return patches_; }

  // Returns a map with positions for the inserted patch
  Patch* TryInsert(const Patch &patch);
  void SetSeeds(const std::vector<Patch> &seeds);

protected:
  Views views_;
  PatchOrganizerOptions options_;
  // This is were patches will actually live
  Patches patches_;
  PatchSet patch_set_;
};

}
}

#endif // DENSEPOINTS_PMVS_PATCH_ORGANIZER
