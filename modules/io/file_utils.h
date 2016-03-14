#ifndef DENSEPOINTS_IO_FILE_UTILS
#define DENSEPOINTS_IO_FILE_UTILS

#include <vector>
#include "cereal/cereal.hpp"
#include "stlplus/file_system.hpp"
#include "core/types.h"

namespace DensePoints {
  namespace IO {

    std::string GetFolder() {
      return "";
    }
    template <typename... Parts>
    std::string GetFolder(std::string folder_part, Parts... folder_parts) {
      const std::string folder = stlplus::folder_append_separator(folder_part) + GetFolder(folder_parts...);
      if (!stlplus::folder_exists(folder)) {
        stlplus::folder_create(folder);
      }
      return folder;
    }

  }
}

#endif
