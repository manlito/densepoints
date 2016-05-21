#ifndef DENSEPOINTS_PMVS_OPTIMIZATION
#define DENSEPOINTS_PMVS_OPTIMIZATION

#include <vector>
#include <opencv2/core.hpp>
#include "patch.h"

namespace DensePoints {
  namespace PMVS {

    class Optimization {
    public:
      Optimization(Patch &patch,
                   std::vector<View> &views,
                   size_t cell_size,
                   double score_threshold = 0.7,
                   size_t minimum_visible_image = 3) :
        patch_(patch),
        views_(views),
        cell_size_(cell_size),
        score_threshold_(score_threshold),
        minimum_visible_image_(minimum_visible_image) { }
      virtual bool Optimize() = 0;
      void GetProjectedTextures(std::vector<cv::Mat> &textures);
      // Patch related filters
      bool FilterByErrorMeasurement();
    private:
      void ParametrizePatch(double &depth, double &roll, double &pitch);
      void UpdatePatch(double depth, double roll, double pitch);

      Patch &patch_;
      std::vector<View> &views_;
      size_t cell_size_;
      size_t minimum_visible_image_;
      double score_threshold_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_OPTIMIZATION
