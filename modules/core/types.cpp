#include "types.h"
#include <opencv2/imgcodecs.hpp>

using namespace DensePoints;

void View::Load()
{
  image_ = cv::imread(image_filename_);
  image_loaded_ = !image_.empty();
}

cv::Mat& View::GetImage()
{
  if (image_loaded_) {
    return image_;
  } else {
    Load();
  }
}

void View::Unload()
{
  image_.release();
  image_loaded_ = false;
}
