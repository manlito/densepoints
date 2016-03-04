#include <Eigen/Geometry>
#include "test_data_generator.h"

ProjectionMatrix TestScene::CreateRandomView(bool randomizeX,
                                             bool randomizeY,
                                             bool randomizeZ)
{
  Matrix3 cameraIntrinsics = Matrix3::Identity();
  // Typical camera
  cameraIntrinsics(0, 0) = 1000;
  cameraIntrinsics(0, 2) = 2000;
  cameraIntrinsics(1, 1) = 1000;
  cameraIntrinsics(1, 2) = 1500;
  ProjectionMatrix cameraExtrinsics = ProjectionMatrix::Identity();

  Matrix3 rotationMatrix = Matrix3::Identity();
  Vector3 translation;
  if (randomizeX) {
    rotationMatrix = rotationMatrix * Eigen::AngleAxis<double>(distribution_angle_(random_generator_), Vector3::UnitX());
    translation[0] = distribution_translation_(random_generator_) + camera_offset_[0];
  }
  if (randomizeY) {
    rotationMatrix = rotationMatrix * Eigen::AngleAxis<double>(distribution_angle_(random_generator_), Vector3::UnitY());
    translation[1] = distribution_translation_(random_generator_) + camera_offset_[1];
  }
  if (randomizeZ) {
    rotationMatrix = rotationMatrix * Eigen::AngleAxis<double>(distribution_angle_(random_generator_), Vector3::UnitZ());
    translation[2] = distribution_translation_(random_generator_) + camera_offset_[2];
  }

  cameraExtrinsics.block<3, 3>(0, 0) = rotationMatrix;
  cameraExtrinsics.col(3) = translation;

  ProjectionMatrix projection_matrix = cameraIntrinsics * cameraExtrinsics;
  projection_matrices_.push_back(projection_matrix);
  return projection_matrix;
}

Vector3 TestScene::CreateRandomPoint(bool randomizeX,
                                     bool randomizeY,
                                     bool randomizeZ)
{
  Vector3 point;
  if (randomizeX) {
    point[0] = distribution_translation_(random_generator_);
  }
  if (randomizeY) {
    point[1] = distribution_translation_(random_generator_);
  }
  if (randomizeZ) {
    point[2] = distribution_translation_(random_generator_);
  }
  points_.push_back(point);
  return point;
}
