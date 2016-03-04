#include "types.h"
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>

using namespace DensePoints;

void View::Load()
{
  image_ = cv::imread(image_filename_);
  image_loaded_ = !image_.empty();
}

cv::Mat& View::GetImage()
{
  if (image_loaded_) {
    return image_;
  } else {
    Load();
  }
}

void View::Unload()
{
  image_.release();
  image_loaded_ = false;
}


void View::SetProjectionMatrix(const ProjectionMatrix &projection_matrix) {
  projection_matrix_ = projection_matrix;

  // Precompute decomposition

  // Camera center: PC = 0
  Eigen::JacobiSVD<Eigen::MatrixX4d> svd(projection_matrix, Eigen::ComputeFullV);
  Vector4 camera_center_homogeneous = svd.matrixV().col(3);
  camera_center_ = camera_center_homogeneous.head(3) / camera_center_homogeneous[3];

  // RQ decomposition from P(:,1:3)
  Matrix3 M = projection_matrix.block<3, 3>(0, 0);
  M.row(0).swap(M.row(2));
  Eigen::HouseholderQR<Matrix3> qr_decomposition(M.transpose());
  Matrix3 R = qr_decomposition.matrixQR();
  Matrix3 Q = qr_decomposition.householderQ();
  R.transposeInPlace();
  R.row(0).swap(R.row(2));
  R.col(0).swap(R.col(2));
  R(1, 0) = 0;
  R(2, 0) = 0;
  R(2, 1) = 0;

  Q.transposeInPlace();
  Q.row(0).swap(Q.row(2));
  ProjectionMatrix E;
  E.block<3, 3>(0, 0) = Q;
  E.col(3) = -Q * camera_center_;

  // Force positive elements on diagonal of K
  Matrix3 signs = Matrix3::Identity();
  for (size_t i = 0; i < 3; ++i) {
    if (R(i, i) < 0) {
      signs(i, i) = -1;
    }
  }

  camera_intrinsics_ = signs * R;
  camera_intrinsics_ /= camera_intrinsics_(2, 2);
  camera_extrinsics_ = signs * E;
}
