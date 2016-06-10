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
                   Views views,
                   size_t cell_size,
                   double score_threshold = 0.6,
                   size_t minimum_visible_image = 3) :
        patch_(patch),
        views_(views),
        cell_size_(cell_size),
        score_threshold_(score_threshold),
        minimum_visible_image_(minimum_visible_image) { }
      virtual bool Optimize() = 0;

      // Get vector of patches
      void GetProjectedTextures(std::vector<cv::Mat> &textures);

      // Used during minimization to search for a better parameter set
      void GetProjectedTextures(const Vector3 normal, const Vector3 position,
                                std::vector<cv::Mat> &textures);
      void ParametrizePatch(const Vector3 normal, const Vector3 position,
                            double &depth, double &roll, double &pitch);
      void ParametrizePatch(double &depth, double &roll, double &pitch);
      void UnparametrizePatch(double depth, double roll, double pitch,
                              Vector3 &normal, Vector3 &position);
      // Patch related filters
      bool FilterByErrorMeasurement();
    protected:

      Patch &patch_;
      Views views_;
      size_t cell_size_;
      size_t minimum_visible_image_;
      double score_threshold_;
    };
  }
}

#endif // DENSEPOINTS_PMVS_OPTIMIZATION
