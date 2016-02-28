#include <string>
#include <iostream>
#include <stdlib.h>
#include "gtest/gtest.h"
#include "test_data_generator.h"
#include "core/types.h"
#include "geometry/triangulation.h"

using namespace DensePoints;

TEST(Core, Triangulation2View) {
  TestScene scene;
  ProjectionMatrix P = scene.CreateRandomView();
  ProjectionMatrix P_p = scene.CreateRandomView();
  Vector4 X;
  X << scene.CreateRandomPoint(), 1;

  Vector3 x_hom = P * X;
  Vector2 x = x_hom.head(2) / x_hom[2];

  Vector3 x_p_hom = P_p * X;
  Vector2 x_p = x_p_hom.head(2) / x_p_hom[2];

  Vector3 X_triangulated = Geometry::DirectLinearTriangulation(P, x, P_p, x_p);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(X[i], X_triangulated[i], 0.01);
  }
}

TEST(Core, TriangulationMultiView) {
  TestScene scene;
  for (size_t i = 0; i < 3; ++i) {
    scene.CreateRandomView();
  }
  auto projection_matrices = scene.GetProjectionMatrices();
  Vector4 X;
  X << scene.CreateRandomPoint(), 1;

  std::vector<Vector2> observations;
  for (const auto &P : projection_matrices) {
    Vector3 x_hom = P * X;
    Vector2 x = x_hom.head(2) / x_hom[2];
    observations.push_back(x);
  }

  Vector3 X_triangulated = Geometry::DirectLinearTriangulation(
                             projection_matrices,
                             observations);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(X[i], X_triangulated[i], 0.01);
  }
}
