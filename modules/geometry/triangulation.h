#ifndef DENSEPOINTS_GEOMETRY_TRIANGULATION
#define DENSEPOINTS_GEOMETRY_TRIANGULATION

#include <vector>
#include <string>
#include "core/types.h"
#include "types.h"

namespace DensePoints {
  namespace Geometry {
    Vector3 DirectLinearTriangulation(const ProjectionMatrix &projection_matrix_1,
                                      const Vector2 &observation_1,
                                      const ProjectionMatrix &projection_matrix_2,
                                      const Vector2 &observation_2);
    Vector3 DirectLinearTriangulation(const std::vector<ProjectionMatrix> &projection_matrices,
                                      const std::vector<Vector2> &observations);
  }
}

#endif // DENSEPOINTS_GEOMETRY_TRIANGULATION
