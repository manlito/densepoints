#ifndef DENSEPOINTS_PMVS_EXPAND
#define DENSEPOINTS_PMVS_EXPAND

#include <vector>
#include "seed.h"
#include "patch_organizer.h"

namespace DensePoints {
  namespace PMVS {
    struct ExpandOptions {
      size_t cell_size;
      ExpandOptions(size_t cell_size = 11)
        : cell_size(cell_size) { }
    };

    class Expand {
    public:
      Expand(Views views,
             ExpandOptions options = ExpandOptions())
        : views_(views),
          options_(options) { }
      void SetOptions(const ExpandOptions expand_options);
      void SetSeeds(const Patches seeds);
      void ExpandPatches();
      Patches ExpandPatch(Patch *const patch);
      // Utility
      void PrintPatchGrids(const std::string folder_name);
    protected:
      Views views_;
      ExpandOptions options_;
      std::unique_ptr<PatchOrganizer> patch_organizer_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_EXPAND
