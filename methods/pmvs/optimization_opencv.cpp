#include <numeric>
#include "optimization_opencv.h"
#include "easylogging/easylogging.h"
#include "core/error_measurements.h"

using namespace DensePoints;
using namespace DensePoints::PMVS;

class PatchOptimizationOpenCVFunctor : public cv::MinProblemSolver::Function {
public:
  PatchOptimizationOpenCVFunctor(OptimizationOpenCV *optimizer)
    : optimizer_(optimizer) {}
  int getDims() const { return 3; }
  double calc(const double *x) const {
    // Expand parameters
    Vector3 normal, position;
    optimizer_->UnparametrizePatch(x[0], x[1], x[2], normal, position);

    // Get texture for all patches
    std::vector<cv::Mat> textures;
    optimizer_->GetProjectedTextures(normal, position, textures);

    // Compute NCC scores
    std::vector<double> scores;
    for (size_t texture_index = 1; texture_index < textures.size(); ++texture_index) {
      // Range will [0, 2] and 0 is more similar
      double score = 1 - NCCScore(textures[0], textures[texture_index]);
      scores.push_back(score);
    }
    // Remove cases with empty scores
    if (scores.size() == 0) {
      return 2;
    }
    // Compute mean
    double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    double mean = sum / scores.size();

    return mean;
  }
private:
  OptimizationOpenCV *optimizer_;
};

bool OptimizationOpenCV::Optimize()
{
  cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
  cv::Ptr<cv::MinProblemSolver::Function> functor =
      cv::makePtr<PatchOptimizationOpenCVFunctor>(this);

  // Initial parameters
  double depth, roll, pitch;
  depth = 0, roll = 0, pitch = 0;

  cv::Mat x = (cv::Mat_<double>(1, 3) << depth, roll, pitch);
  // TODO: This initial step is really sensitive... need an alternative
  cv::Mat initial_step = (cv::Mat_<double>(3, 1)<< 0.0006, 0.08, 0.08);

  solver->setFunction(functor);
  solver->setInitStep(initial_step);
  cv::TermCriteria term_criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 0.01);
  solver->setTermCriteria(term_criteria);

  double res = solver->minimize(x);

  // Expand and update parameters
  Vector3 normal, position;
  UnparametrizePatch(x.at<double>(0, 0), x.at<double>(0, 1), x.at<double>(0, 2),
                     normal, position);
  patch_.SetNormal(normal);
  patch_.SetPosition(position);
#ifdef DEBUG_PMVS_OPTIMIZATION
  LOG(INFO) << "Optimization END with: "
            << x.at<double>(0, 0) << ", "
            << x.at<double>(0, 1) << ", "
            << x.at<double>(0, 2);
#endif
  return true;
}
