#ifndef DENSEPOINTS_PMVS_OPTIMIZATION_OPENCV
#define DENSEPOINTS_PMVS_OPTIMIZATION_OPENCV

#include <vector>
#include <opencv2/core.hpp>
#include "optimization.h"

namespace DensePoints {
  namespace PMVS {
    class OptimizationOpenCV : public Optimization {
    public:
      using Optimization::Optimization;
      bool Optimize();
    };
  }
}

#endif // DENSEPOINTS_PMVS_OPTIMIZATION_OPENCV
