#ifndef DENSEPOINTS_PMVS_OPTIONS
#define DENSEPOINTS_PMVS_OPTIONS

#include <vector>

namespace DensePoints {
  namespace PMVS {
    class Options {
    public:
      Options(const int scale = 1,
              const int cell_size = 4,
              const int expansions = 3) :
        scale_(scale),
        cell_size_(cell_size),
        expansions_(expansions) {}

    protected:
      int scale_;
      int cell_size_;
      int expansions_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_OPTIONS
