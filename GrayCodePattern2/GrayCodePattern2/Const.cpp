#include "Const.h";
#include <opencv2/core.hpp>;
using namespace cv;
extern const int proj_width = 1024;
extern const int proj_height = 768;

extern const char* images_group_dir = "partten_images%02d\\";
extern const int images_group_dir_length = 18;
extern const String images_file = "pattern_cam1_im";
extern const String imagesName_file = "imgName.yaml";
extern const String imgType = ".jpg";

extern const bool isThresh = true;
extern const size_t white_thresh = 0;
extern const size_t black_thresh = 1;

extern const bool isRemap = true;

// Optimize disparity map
extern const bool isOptimize = true;
extern const float downPortion = 0.09;
extern const float upPortion = 0.98;
