#ifndef DENSEPOINTS_IO_FILE_UTILS
#define DENSEPOINTS_IO_FILE_UTILS

#include <vector>
#include <initializer_list>
#include "cereal/cereal.hpp"
#include "stlplus/file_system.hpp"
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

#endif
