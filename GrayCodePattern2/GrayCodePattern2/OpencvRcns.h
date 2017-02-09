#ifndef OPENCVRCNS_H
#define OPENCVRCNS_H
#include <opencv2/structured_light.hpp>
#include <opencv2/core.hpp>
#include "VirtualCamera.h"
using namespace cv;
using namespace std;
class OpencvRcns
{
public:
	static void decodeTwoGroupOfImg(const Ptr<structured_light::GrayCodePattern>& graycode, const vector<string>& imagelist, VirtualCamera& camera1, VirtualCamera& camera2, const int count, Mat& pointcloud_tresh, Mat& color_tresh);
};
#endif