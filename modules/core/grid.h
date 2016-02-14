#ifndef DENSEPOINTS_CORE_GRID
#define DENSEPOINTS_CORE_GRID

namespace DensePoints {
  class Grid {
  public:
    Grid(size_t cell_size,
         size_t image_width,
         size_t image_height) :
      cell_size_(cell_size),
      image_width_(image_width),
      image_height_(image_height)
    {
      grid_width_ = image_width_ / cell_size_;
      if (image_width_ % cell_size_)
        grid_width_++;

      grid_height_ = image_height_ / cell_size_;
      if (image_height_ % cell_size_)
        grid_height_++;
    }

    size_t Rows();
    size_t Columns();
    size_t Size();
    size_t CellX(size_t input_x);
    size_t CellY(size_t input_y);
    size_t CellXY(size_t input_x, size_t input_y);

  protected:
    size_t cell_size_;
    size_t image_width_;
    size_t image_height_;
    size_t grid_width_;
    size_t grid_height_;
  };
}

#endif // DENSEPOINTS_CORE_GRID
