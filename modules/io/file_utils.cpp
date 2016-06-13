#include "file_utils.h"
#include <vector>
#include <initializer_list>
#include "cereal/cereal.hpp"
#include "core/types.h"

namespace DensePoints {
namespace IO {

std::string GetFolder(std::initializer_list<std::string> parts) {
  std::string folder = "";
  if (parts.size() > 0) {
    for (const auto &part : parts) {
      folder += stlplus::folder_append_separator(part);
      if (!stlplus::folder_exists(folder)) {
        stlplus::folder_create(folder);
      }
    }
  }
  return folder;
}

}
}
