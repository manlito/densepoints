#ifndef DENSEPOINTS_GEOMETRY_TYPES
#define DENSEPOINTS_GEOMETRY_TYPES

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "core/types.h"

namespace DensePoints {
  namespace Geometry {
    typedef Eigen::ParametrizedLine<double, 2> Line2D;
  }
}

#endif // DENSEPOINTS_GEOMETRY_TYPES
