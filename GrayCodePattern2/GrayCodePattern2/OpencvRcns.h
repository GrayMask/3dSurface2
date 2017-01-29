#ifndef OPENCVRCNS_H
#define OPENCVRCNS_H
#include <opencv2/structured_light.hpp>
#include <opencv2/core.hpp>
using namespace cv;
using namespace std;
class OpencvRcns
{
public:
	static void decodeTwoGroupOfImg(const Ptr<structured_light::GrayCodePattern>& graycode, const vector<string>& imagelist, const Mat& intrinsics, const Mat& distCoeffs, const Mat& R, const Mat& T, const int count, Mat& pointcloud_tresh, Mat& color_tresh);
};
#endif