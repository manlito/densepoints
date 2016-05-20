#include "optimization_opencv.h"

using namespace DensePoints;
using namespace DensePoints::PMVS;

class PatchOptimizationOpenCVFunctor : public cv::MinProblemSolver::Function {
public:
  int getDims() const { return 3; }
  double calc(const double *x) const {

  }
};

bool OptimizationOpenCV::Optimize()
{
  cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
  cv::Ptr<cv::MinProblemSolver::Function> functor =
      cv::makePtr<PatchOptimizationOpenCVFunctor>();

  double depth, roll, pitch;
  void ParametrizePatch(double &depth, double &roll, double &pitch);

  cv::Mat x = (cv::Mat_<double>(1, 3) << depth, roll, pitch);
  cv::Mat initial_step = (cv::Mat_<double>(3, 1)<< -0.01, -0.1, -0.1);

  solver->setInitStep(initial_step);

  double res = solver->minimize(x);

  return true;
}
