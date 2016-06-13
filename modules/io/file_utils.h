#ifndef DENSEPOINTS_IO_FILE_UTILS
#define DENSEPOINTS_IO_FILE_UTILS

#include <string>
#include <initializer_list>
#include "stlplus/file_system.hpp"

namespace DensePoints {
namespace IO {

std::string GetFolder(std::initializer_list<std::string> parts);

}
}

#endif
