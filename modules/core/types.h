#ifndef DENSEPOINTS_CORE_TYPES
#define DENSEPOINTS_CORE_TYPES

#include <vector>
#include <string>
#include <thread>
#include <eigen3/Eigen/Core>
#include <pcl/common/common.h>
#include <opencv2/core.hpp>

namespace DensePoints {

  // Used for math operations
  typedef Eigen::Matrix<double, 3, 4> Matrix34;
  typedef Eigen::Matrix<double, 4, 3> Matrix43;
  typedef Eigen::Matrix4d Matrix4;
  typedef Eigen::Matrix3d Matrix3;
  typedef Eigen::Matrix3d FundamentalMatrix;
  typedef Matrix34 ProjectionMatrix;
  typedef Eigen::Vector2d Vector2;
  typedef Eigen::Vector3d Vector3;
  typedef Eigen::Vector4d Vector4;

  // Used for representation
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointCloudXYZRGBNormal;

  // Matching
  typedef std::pair<size_t, size_t> ImagesPair;
  typedef std::vector<ImagesPair> ImagesPairsList;

  // ROI
  typedef cv::Rect Rectangle;

  class View {
  public:
    View(const ProjectionMatrix &projection_matrix) { SetProjectionMatrix(projection_matrix); }
    View(std::string image_filename) :
      image_filename_(image_filename),
      image_loaded_(false) {}

    cv::Mat& GetImage();
    bool ImageLoaded() const { return image_loaded_; }
    std::string GetImageFilename() const { return image_filename_; }
    void Load();
    void Unload();

    void SetProjectionMatrix(const ProjectionMatrix &projection_matrix);
    ProjectionMatrix GetProjectionMatrix() const { return projection_matrix_; }
    Matrix3 GetIntrinsics() const { return camera_intrinsics_; }
    ProjectionMatrix GetExtrinsics() const { return camera_extrinsics_; }
    Vector3 GetCameraCenter() const { return camera_center_; }
    Vector2 ProjectPoint(const Vector3 &point) const;
    Vector3 GetXAxis() const;

    size_t Height() const { return image_.rows; }
    size_t Width() const { return image_.cols; }

    bool IsPointInside(const Vector3 &point) const;

  private:
    ProjectionMatrix projection_matrix_;

    // Calculated using decomposition of projection matrix
    ProjectionMatrix camera_extrinsics_;
    Matrix3 camera_intrinsics_;
    Vector3 camera_center_;

    std::string image_filename_;
    bool image_loaded_;
    cv::Mat image_;
  };

}
#endif // DENSEPOINTS_CORE_TYPES
