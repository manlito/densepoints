#include "expand.h"

namespace DensePoints {
namespace PMVS {

void Expand::SetSeeds(const Patches seeds)
{
  patch_organizer_.reset();
  patch_organizer_ = std::unique_ptr<PatchOrganizer>(new PatchOrganizer(views_));
  patch_organizer_->AllocateViews();
  patch_organizer_->SetSeeds(seeds);
}

}
}
