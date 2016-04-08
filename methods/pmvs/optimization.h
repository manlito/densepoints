#ifndef DENSEPOINTS_PMVS_OPTIMIZATION
#define DENSEPOINTS_PMVS_OPTIMIZATION

#include <vector>
#include <opencv2/core.hpp>
#include "patch.h"

namespace DensePoints {
  namespace PMVS {
    class Optimization {
    public:
      Optimization(Patch &patch, const std::vector<View> &views) :
        patch_(patch),
        views_(views) {}
      bool Optimize();
      void GetProjectedTextures(std::vector<cv::Mat> &textures) const;
    private:
      virtual bool OptimizePatch(std::vector<cv::Mat> &textures,
                                 double &depth,
                                 double &roll,
                                 double &pitch) = 0;
      void ParametrizePatch(double &depth, double &roll, double &pitch);
      void UpdatePatch(double depth, double roll, double pitch);

      Patch &patch_;
      const std::vector<View> &views_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_OPTIMIZATION
