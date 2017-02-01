#include "Const.h";
#include <opencv2/core.hpp>;
using namespace cv;
extern const int proj_width = 1024;
extern const int proj_height = 768;

extern const int cam_width = 4896;
extern const int cam_height = 3264;

// dir
extern const String exprNum = "7";
extern const String expr_dir = "expr" + exprNum + "\\";
extern const String calib_file = "calibration_result\\camera2.xml";
extern const String disparityMap_file = "matlab\\disparityMap" + exprNum;
extern const String groupNum_file = expr_dir + "groupNum.txt";
extern const char* images_group_dir = "partten_images%02d\\";
extern const int images_group_dir_length = 18;
extern const String images_file = "pattern_im";
extern const String imagesName_file = "imgName.yaml";
extern const String imgType = ".jpg";
extern const String sfm_dir = expr_dir + "sfm_images\\";
extern const String sfm_file = sfm_dir + "d.nvm.cmvs\\00\\cameras_v2.txt";
extern const String ply_file = "pointCloud.ply";

extern const bool isThresh = true;
extern const size_t white_thresh = 5;
extern const size_t black_thresh = 40;

extern const bool isRemap = true;

// Optimize disparity map
extern const bool isOptimize = true;
extern const float downPortion = 0.002;
extern const float upPortion = 0.99;

extern const bool isShowResult = false;

extern const bool isUnderWorld = false;