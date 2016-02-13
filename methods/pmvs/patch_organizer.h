#ifndef DENSEPOINTS_PMVS_PATCH_ORGANIZER
#define DENSEPOINTS_PMVS_PATCH_ORGANIZER

#include <vector>
#include "patch.h"

namespace DensePoints {
  namespace PMVS {

    typedef std::vector<Patch> PatchGridCell;

    class PatchGrid {
      PatchGrid(size_t width, size_t height, size_t max_patches_per_cell);
      const PatchCell& GetAt(size_t row, size_t column) const;
      void Insert(size_t row, size_t column, Patch patch);

    private:
      std::vector<PatchGridCell> grid_;
    };

    class PatchOrganizer {
    public:
    private:
      PatchGrid patch_grid_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_PATCH_ORGANIZER
