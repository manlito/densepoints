#include <fstream>
#include "json_reader.h"
#include "cereal/archives/json.hpp"
#include "cereal/types/vector.hpp"

using namespace DensePoints;
using namespace DensePoints::IO;

JSONReader::JSONReader(std::string filename)
{
  std::ifstream input_stream(filename);
  cereal::JSONInputArchive archive(input_stream);

  std::string images_path;
  std::vector<ViewArchive> views;
  archive(cereal::make_nvp("imagesPath", images_path),
          cereal::make_nvp("views", views));

  for (const ViewArchive viewArchive : views) {
    View view(stlplus::create_filespec(images_path, viewArchive.filename));
    view.projection_matrix <<
        viewArchive.projection_matrix[0][0], viewArchive.projection_matrix[0][1], viewArchive.projection_matrix[0][2], viewArchive.projection_matrix[0][3],
        viewArchive.projection_matrix[1][0], viewArchive.projection_matrix[1][1], viewArchive.projection_matrix[1][2], viewArchive.projection_matrix[1][3],
        viewArchive.projection_matrix[2][0], viewArchive.projection_matrix[2][1], viewArchive.projection_matrix[2][2], viewArchive.projection_matrix[2][3];
    views_.push_back(view);
  }
}

std::vector<View> JSONReader::GetViews()
{
  return views_;
}
