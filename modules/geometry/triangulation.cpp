#include <Eigen/Dense>
#include "triangulation.h"

using namespace DensePoints;

Vector3 Geometry::DirectLinearTriangulation(const ProjectionMatrix &projection_matrix_1,
                                            const Vector2 &observation_1,
                                            const ProjectionMatrix &projection_matrix_2,
                                            const Vector2 &observation_2)
{
  return Geometry::DirectLinearTriangulation(std::vector<ProjectionMatrix> {projection_matrix_1, projection_matrix_2},
                                             std::vector<Vector2> {observation_1, observation_2});
}

Vector3 Geometry::DirectLinearTriangulation(const std::vector<ProjectionMatrix> &projection_matrices,
                                            const std::vector<Vector2> &observations)
{
  // H.Z. Page 312, extended to multiple views
  Eigen::MatrixX4f A(projection_matrices.size() * 2, 4);
  for (size_t i = 0; i < projection_matrices.size(); ++i) {
    const ProjectionMatrix &P = projection_matrices[i];
    const float x = observations[i][0];
    const float y = observations[i][1];
    A.row(i * 2) = x * P.row(2) - P.row(0);
    A.row(i * 2 + 1) = y * P.row(2) - P.row(1);
  }

  // Nullspace of A
  Eigen::JacobiSVD<Eigen::MatrixX4f> svd(A, Eigen::ComputeFullV);
  Vector4 X = svd.matrixV().col(3);

  // Return inhomogeneous
  return Vector3(X.head(3) / X[3]);
}
