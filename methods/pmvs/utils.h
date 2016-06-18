#ifndef DENSEPOINTS_PMVS_UTILS
#define DENSEPOINTS_PMVS_UTILS

#include <vector>
#include <map>
#include "core/types.h"
#include "patch.h"

namespace DensePoints {
namespace PMVS {

void PrintCloud(const Patches patches,
                const std::string folder_name,
                const std::string file_name);

}
}

#endif // DENSEPOINTS_PMVS_UTILS
