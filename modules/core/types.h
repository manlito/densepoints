#ifndef DENSEPOINTS_CORE_TYPES
#define DENSEPOINTS_CORE_TYPES

#include <string>
#include <eigen3/Eigen/Core>
#include <pcl/common/common.h>
#include <opencv2/core.hpp>

namespace DensePoints {

  // Used for math operations
  typedef Eigen::Matrix<double, 3, 4> Mat34;
  typedef Mat34 ProjectionMatrix;
  typedef Eigen::Vector2f Vector2;
  typedef Eigen::Vector3f Vector3;
  typedef Eigen::Vector4f VectorHomogeneous;

  // Used for representation
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZ> PointCloudXYZRGBNormal;

  class View {
  public:
    View(std::string image_filename) :
      image_filename_(image_filename),
      image_loaded_(false) {}

    cv::Mat& GetImage();
    bool ImageLoaded() const { return image_loaded_; }
    std::string GetImageFilename() const { return image_filename_; }
    void Load();
    void Unload();

    ProjectionMatrix projection_matrix;

  private:
    std::string image_filename_;
    bool image_loaded_;
    cv::Mat image_;
  };

}

#endif // DENSEPOINTS_CORE_TYPES
