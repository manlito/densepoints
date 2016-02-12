#ifndef DENSEPOINTS_PMVS_OPTIONS
#define DENSEPOINTS_PMVS_OPTIONS

#include <vector>

namespace DensePoints {
  namespace PMVS {
    class Options {
    public:
      Options(const int cell_size = 4,
              const int expansions = 3) :
        cell_size_(cell_size),
        expansions_(expansions) {}

    protected:
      int cell_size_;
      int expansions_;
    };
  }
}

#endif DENSEPOINTS_PMVS_OPTIONS
