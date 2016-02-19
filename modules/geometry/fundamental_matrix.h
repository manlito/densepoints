#ifndef DENSEPOINTS_GEOMETRY_FUNDAMENTAL_MATRIX
#define DENSEPOINTS_GEOMETRY_FUNDAMENTAL_MATRIX

#include <vector>
#include <string>
#include "core/types.h"
#include "types.h"

namespace DensePoints {
  namespace Geometry {
    FundamentalMatrix ComputeFundamentalMatrix(const ProjectionMatrix &projection_matrix_1,
                                               const ProjectionMatrix &projection_matrix_2);
    Line2D LineFromFundamentalMatrix(FundamentalMatrix &fundamental_matrix, Vector2 point);
    float YCoordinateAt(Line2D &line, float x_coordinate);
  }
}

#endif // DENSEPOINTS_GEOMETRY_FUNDAMENTAL_MATRIX
