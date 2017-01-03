#include <opencv2/core.hpp>;
using namespace cv;

extern const int proj_width;
extern const int proj_height;

// dir
extern const String exprNum;
extern const String expr_dir;
extern const String calib_file;
extern const String disparityMap_file;
extern const String groupNum_file;
extern const char* images_group_dir;
extern const int images_group_dir_length;
extern const String images_file;
extern const String imagesName_file;
extern const String imgType;
extern const String sfm_dir;
extern const String sfm_file;
extern const String ply_file;

extern const bool isThresh;
extern const size_t white_thresh;
extern const size_t black_thresh;

extern const bool isRemap;

// Optimize disparity map
extern const bool isOptimize;
extern const float downPortion;
extern const float upPortion;

extern const bool isShowResult;