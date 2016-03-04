#include <Eigen/Dense>
#include "fundamental_matrix.h"

using namespace DensePoints;

FundamentalMatrix Geometry::ComputeFundamentalMatrix(const ProjectionMatrix &projection_matrix_1,
                                                     const ProjectionMatrix &projection_matrix_2)

{
  // H.Z. Page 244
  // Variables to have same notation as book
  const ProjectionMatrix &P = projection_matrix_1;
  const ProjectionMatrix &P_p = projection_matrix_2;

  // C (for PC = 0)
  Eigen::FullPivLU<ProjectionMatrix> lu_decomp(P);
  const Vector4 C = lu_decomp.kernel();

  // e' = P'C
  const Vector3 e_p = P_p * C;

  // P^+, such that PP^+ = I
  Matrix43 P_inv = P.adjoint() * (P * P.adjoint()).inverse();

  // [e']_x (A4.5)
  Matrix3 e_p_x;
  e_p_x <<       0, -e_p[2],  e_p[1],
            e_p[2],       0, -e_p[0],
           -e_p[1],  e_p[0],       0;

  // F =  [e']_x * P' * P^+
  FundamentalMatrix F = e_p_x * P_p * P_inv;

  return F;
}

Geometry::Line2D Geometry::LineFromFundamentalMatrix(FundamentalMatrix &fundamental_matrix, Vector2 point)
{
  // Compute slope
  Vector3 point_homogeneous;
  point_homogeneous << point[0], point[1], 1;

  // Line as: ax + by + c =0
  Vector3 projective_line = fundamental_matrix * point_homogeneous;
  float y_1, y_2;

  // Assume: x_1 = 0, x_2 = 1
  y_1 = -projective_line[2] / projective_line[1];
  y_2 = (-projective_line[2] - projective_line[0]) / projective_line[1];

  Line2D line = Line2D::Through(Vector2(0, y_1), Vector2(1, y_2));
  return line;
}

double Geometry::YCoordinateAt(Line2D &line, double x_coordinate)
{
  Eigen::Hyperplane<double, 2> hp(Vector2(1, 0), -x_coordinate);
  Vector2 intersection = line.intersectionPoint(hp);
  return intersection[1];
}
