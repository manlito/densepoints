#include <opencv2/imgcodecs.hpp>
#include "pmvs.h"
#include "seed.h"

using namespace DensePoints::PMVS;

void PMVS::AddCamera(const ProjectionMatrix &projection_matrix,
                     const std::string filename)
{
  cv::Mat image = cv::imread(filename);

  if (!image.empty()) {

  } else {
  }
}

bool PMVS::Run()
{
  InsertSeeds();

  return true;
}

void PMVS::InsertSeeds()
{
  std::vector<Vector3> seeds;
  Seed seed;
  seed.GenerateSeeds(images_, seeds);
}
