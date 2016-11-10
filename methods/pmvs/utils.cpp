#include "utils.h"
#include "io/file_utils.h"
#include "rplycpp/src/rplycpp.hpp"
#include <opencv2/calib3d.hpp>

using namespace DensePoints;
using namespace DensePoints::PMVS;

void PMVS::PrintCloud(const Patches patches,
                      const std::string folder_name,
                      const std::string file_name)
{
  // Debug patches
  rplycpp::PLYWriter writer;
  writer.Open(stlplus::create_filespec(IO::GetFolder({ DEBUG_OUTPUT_PATH, folder_name }),
                                       file_name, "ply"));
  std::vector<rplycpp::PLYProperty> properties;
  rplycpp::PLYProperty property;
  property.type = rplycpp::PLYDataType::PLY_FLOAT;
  property.name = "x";
  properties.push_back(property);
  property.name = "y";
  properties.push_back(property);
  property.name = "z";
  properties.push_back(property);
  property.type = rplycpp::PLYDataType::PLY_UCHAR;
  property.name = "red";
  properties.push_back(property);
  property.name = "green";
  properties.push_back(property);
  property.name = "blue";
  properties.push_back(property);
  property.type = rplycpp::PLYDataType::PLY_FLOAT;
  property.name = "nx";
  properties.push_back(property);
  property.name = "ny";
  properties.push_back(property);
  property.name = "nz";
  properties.push_back(property);
  writer.AddElement("vertex", properties, patches.size());

  // Add the data IN THE SAME ORDER!
  for (const Patch &patch : patches) {
    const PointXYZRGBNormal point = patch.GetPoint();
    writer.AddRow(std::vector<double> { point.x, point.y, point.z,
                                        point.r, point.g, point.b,
                                        point.normal_x, point.normal_y, point.normal_z});
  }
  writer.Close();
}
