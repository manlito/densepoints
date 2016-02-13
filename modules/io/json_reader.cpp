#include <fstream>
#include "json_reader.h"
#include "cereal/archives/json.hpp"
#include "cereal/types/vector.hpp"

using namespace DensePoints::IO;

JSONReader::JSONReader(std::string filename)
{
  std::ifstream input_stream(filename);
  cereal::JSONInputArchive archive(input_stream);

  archive(cereal::make_nvp("imagesPath", images_path_),
          cereal::make_nvp("views", views_));
}

std::string JSONReader::GetImagesPath()
{
  return images_path_;
}

std::vector<ViewArchive> JSONReader::GetViews()
{
  return views_;
}
