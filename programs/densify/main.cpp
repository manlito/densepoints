#include <stdlib.h>
#include "easylogging/easylogging.h"
#include "cxxopts/cxxopts.hpp"

#include "pmvs/pmvs.h"
#include "io/json_reader.h"

INITIALIZE_EASYLOGGINGPP

using namespace DensePoints;

int main(int argc, char *argv[])
{
  LOG(INFO) << "Starting PMVS";

  cxxopts::Options options(argv[0], " - Command line options");
  options.add_options()
    ("i,input", "Input JSON scene file", cxxopts::value<std::string>())
    ("s,settings", "Settings JSON file", cxxopts::value<std::string>()->default_value(""));
  options.parse(argc, argv);

  if (options.count("input") < 1) {
    LOG(FATAL) << "Missing input scene filename";
  }

  std::string scene_filename = options["input"].as<std::string>();
  std::string settings_filename = options["settings"].as<std::string>();

  LOG(INFO) << "Reading " << scene_filename;
  IO::JSONReader reader(scene_filename);

  PMVS::PMVS pmvs;
  for (const View view : reader.GetViews()) {
    pmvs.AddCamera(view);
  }
  pmvs.Run();

  LOG(INFO) << "Finished PMVS";
  return EXIT_SUCCESS;
}
