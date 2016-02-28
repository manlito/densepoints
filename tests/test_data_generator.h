#ifndef TEST_DATA_GENERATOR
#define TEST_DATA_GENERATOR
#include <string>
#include "core/types.h"
#include <random>

using namespace DensePoints;

class TestScene {
public:
  TestScene(float angle_spread = 90,
            float translation_spread = 10,
            Vector3 camera_offset = Vector3(0, 0, -20)) :
            angle_spread_(angle_spread),
            translation_spread_(translation_spread),
            camera_offset_(camera_offset)
  {
    // Generators for random numbers
    std::random_device rd;
    random_generator_ = std::mt19937(rd());
    distribution_angle_ = std::uniform_real_distribution<>(-angle_spread, angle_spread);
    distribution_translation_ = std::uniform_real_distribution<>(0, translation_spread);
  }
  ProjectionMatrix CreateRandomView(bool randomizeX = true,
                                    bool randomizeY = true,
                                    bool randomizeZ = true);
  Vector3 CreateRandomPoint(bool randomizeX = true,
                            bool randomizeY = true,
                            bool randomizeZ = true);
  std::vector<ProjectionMatrix> GetProjectionMatrices() { return projection_matrices_;}
private:
  std::vector<ProjectionMatrix> projection_matrices_;
  std::vector<Vector3> points_;
  // Settings for random numbers
  float angle_spread_;
  float translation_spread_;
  Vector3 camera_offset_;
  // Generation of random numbers
  std::mt19937 random_generator_;
  std::uniform_real_distribution<> distribution_angle_;
  std::uniform_real_distribution<> distribution_translation_;
};

#endif
