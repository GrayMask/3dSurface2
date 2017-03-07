#include "Const.h";
#include <opencv2/core.hpp>;

extern const int proj_width = 1024;
extern const int proj_height = 768;

extern const int cam_width = 1280;// 1280;//800;//4896;
extern const int cam_height = 960;// 720;//600;//3264;
extern const int cam_exp = 1;

extern const bool isStereoCamera = true;

// dir
extern const cv::String exprNum = "14";
extern const cv::String expr_dir = "expr" + exprNum + "\\";
extern const cv::String calib_file = "calibration_result\\camera4.xml";
extern const cv::String disparityMap_file = "matlab\\disparityMap" + exprNum;
extern const cv::String groupNum_file = expr_dir + "groupNum.txt";
extern const char* images_group_dir = "partten_images%02d\\";
extern const int images_group_dir_length = 18;
extern const cv::String images_file = "pattern_im";
extern const cv::String imagesName_file = "imgName.yaml";
extern const cv::String imgType = ".jpg";
extern const cv::String sfm_dir = expr_dir + "sfm_images\\";
extern const cv::String sfm_file = sfm_dir + "d.nvm.cmvs\\00\\cameras_v2.txt";
extern const cv::String ply_file = "pointCloud.ply";

extern const bool isThresh = true;
extern const size_t white_thresh = 5;
extern const size_t black_thresh = 40;

extern const bool isRemap = true;

// Optimize disparity map
extern const bool isOptimize = true;
extern const float downPortion = 0.001;
extern const float upPortion = 0.999;

extern const bool isShowResult = false;

extern const bool isUnderWorld = false;