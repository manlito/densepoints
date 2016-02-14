#ifndef DENSEPOINTS_IO_JSON
#define DENSEPOINTS_IO_JSON

#include <vector>
#include "cereal/cereal.hpp"
#include "stlplus/file_system.hpp"
#include "core/types.h"

namespace DensePoints {
  namespace IO {

    struct ViewArchive {
      std::string filename;
      std::vector<std::vector<double>> projection_matrix;
      ViewArchive() {
        // Allocate for a 3x4 matrix
        projection_matrix.resize(3);
        for (auto & row : projection_matrix) {
          row.resize(4);
        }
      }

      template <typename Archive>
      void serialize(Archive &ar) {
        ar(CEREAL_NVP(filename),
           cereal::make_nvp("projectionMatrix", projection_matrix));
      }
    };

    class JSONReader {
    public:
      JSONReader(std::string filename);
      std::string GetImagesPath();
      std::vector<View> GetViews();
    protected:
      std::vector<View> views_;
    };

  }
}

#endif
