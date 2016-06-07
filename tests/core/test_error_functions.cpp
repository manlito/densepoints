#include <string>
#include <iostream>
#include <stdlib.h>
#include "gtest/gtest.h"
#include "core/error_measurements.h"

using namespace DensePoints;

TEST(ErrorFunctions, NCCScore) {
  cv::Mat a = (cv::Mat_<double>(3, 3) << 1, 2, 3, -1, -2, -3, 1, 2, 3);
  cv::Mat b = (cv::Mat_<double>(3, 3) << 2, 0, 5, -4, 5, -2, -1, 0, -3);

  EXPECT_FLOAT_EQ(NCCScore(a, b), 0.1005653);
  EXPECT_FLOAT_EQ(NCCScore(a, a), 1.0);
}
