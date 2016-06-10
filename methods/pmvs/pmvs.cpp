#include <opencv2/imgcodecs.hpp>
#include "easylogging/easylogging.h"
#include "pmvs.h"
#include "seed.h"

namespace DensePoints {
namespace PMVS {

void PMVS::AddCamera(View view)
{
  view.Load();

  if (view.ImageLoaded()) {
    views_->push_back(view);
  } else {
    LOG(WARNING) << "View with image " << view.GetImageFilename() << " discarded";
  }
}

bool PMVS::Run()
{
  InsertSeeds();
  ExpandSeeds();
  return true;
}

void PMVS::InsertSeeds()
{
  seeds_ = std::make_shared<Seed>(views_);
  seeds_->GenerateSeeds();
}

void PMVS::ExpandSeeds()
{
  std::vector<Patch> seeds;
  seeds_->GetPatches(seeds);

  expand_ = std::make_shared<Expand>(views_);
  expand_->SetSeeds(seeds);
}


}
}

