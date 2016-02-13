#ifndef DENSEPOINTS_CORE_TYPES
#define DENSEPOINTS_CORE_TYPES

#include <eigen3/Eigen/Core>
#include <pcl/common/common.h>

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

#endif // DENSEPOINTS_CORE_TYPES
