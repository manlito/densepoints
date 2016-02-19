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
  typedef Eigen::Matrix<float, 3, 4> Matrix34;
  typedef Eigen::Matrix<float, 4, 3> Matrix43;
  typedef Eigen::Matrix4f Matrix4;
  typedef Eigen::Matrix3f Matrix3;
  typedef Eigen::Matrix3f FundamentalMatrix;
  typedef Matrix34 ProjectionMatrix;
  typedef Eigen::Vector2f Vector2;
  typedef Eigen::Vector3f Vector3;
  typedef Eigen::Vector4f Vector4;

  // Used for representation
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZ> PointCloudXYZRGBNormal;

  // Matching
  typedef std::pair<size_t, size_t> ImagesPair;
  typedef std::vector<ImagesPair> ImagesPairsList;


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
