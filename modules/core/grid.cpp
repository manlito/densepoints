#include "types.h"
#include "grid.h"
#include <opencv2/imgcodecs.hpp>

using namespace DensePoints;

size_t Grid::Columns()
{
  return grid_width_;
}

size_t Grid::Rows()
{
  return grid_height_;
}

size_t Grid::Size()
{
  return Columns() * Rows();
}

size_t Grid::CellX(size_t input_x)
{
  return input_x / cell_size_;
}

size_t Grid::CellY(size_t input_y)
{
  return input_y / cell_size_;
}

size_t Grid::CellXY(size_t input_x, size_t input_y)
{
  return (input_y / cell_size_) * Columns() + input_x / cell_size_;
}
