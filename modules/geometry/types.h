#ifndef DENSEPOINTS_GEOMETRY_TYPES
#define DENSEPOINTS_GEOMETRY_TYPES

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_3.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/IO/Color.h>

typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Mat34 ProjectionMatrix;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector4f VectorHomogeneous;

typedef CGAL::Simple_cartesian<double>::Point_3 Point3;
typedef CGAL::Point_with_normal_3<Simple_cartesian<double>> Point3Normal;
typedef CGAL::Color Color;

#endif DENSEPOINTS_GEOMETRY_TYPES
