#include <opencv2/imgcodecs.hpp>
#include "easylogging/easylogging.h"
#include "pmvs.h"
#include "seed.h"

using namespace DensePoints::PMVS;

void PMVS::AddCamera(View view)
{
  view.Load();

  if (view.ImageLoaded()) {
    views_.push_back(view);
  } else {
    LOG(WARNING) << "View with image " << view.GetImageFilename() << " discarded";
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
  Seed seed(views_);
  seed.GenerateSeeds(seeds);
}
