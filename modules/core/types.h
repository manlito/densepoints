#ifndef DENSEPOINTS_CORE_TYPES
#define DENSEPOINTS_CORE_TYPES

#include <string>
#include <thread>
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

  class Threading {
  public:
    template<typename Callable, typename... Arguments>
    static void Run(size_t thread_count,
                    Callable&& thread_function,
                    Arguments&&... arguments)
    {
      std::vector<std::thread> threads;
      for (size_t i = 0; i < thread_count; ++i) {
        threads.push_back(std::thread(thread_function, std::forward<Arguments>(arguments)...));
      }
      for (std::thread &thread : threads) {
        thread.join();
      }
      threads.clear();
    }
  };
}

#endif // DENSEPOINTS_CORE_TYPES
