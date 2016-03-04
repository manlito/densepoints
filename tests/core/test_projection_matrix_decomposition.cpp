#include <string>
#include <iostream>
#include <stdlib.h>
#include "gtest/gtest.h"
#include "test_data_generator.h"
#include "core/types.h"

using namespace DensePoints;

TEST(Core, ProjectionMatrixDecomposition) {
  TestScene scene;
  ProjectionMatrix P;
  P << 3.53553e2, 3.39645e2, 2.77744e2, -1.44946e6,
      -1.03528e2, 2.33212e1, 4.59607e2, -6.32525e5,
      7.07107e-1, -3.53553e-1, 6.12372e-1, -9.18559e2;
  std::cout << "P: " << std::endl << P << std::endl;

  View view(P);
  Matrix3 intrinsics = view.GetIntrinsics();
  ProjectionMatrix extrinsics = view.GetExtrinsics();
  ProjectionMatrix recovered_matrix = intrinsics * extrinsics;
  EXPECT_NEAR(intrinsics(0, 0), 468.2, 0.1) << "Incorrect fx";
  EXPECT_NEAR(intrinsics(1, 1), 427.2, 0.1) << "Incorrect fy";
  EXPECT_NEAR(intrinsics(0, 2), 300, 0.1) << "Incorrect cx";
  EXPECT_NEAR(intrinsics(1, 2), 200, 0.1) << "Incorrect cy";
  EXPECT_NEAR(intrinsics(2, 2), 1, 0.1) << "K(2,2) should be 1";
  for (size_t row = 0; row < 3; ++row) {
    for (size_t col = 0; col < 4; ++col) {
      EXPECT_NEAR(recovered_matrix(row, col), P(row, col), 0.5)
          << "(" << row << ", " << col << ") does not match";
    }
  }
  Vector3 center = view.GetCameraCenter();
}
