#ifndef UNDERWORLDRCNS_H
#define UNDERWORLDRCNS_H
#include <opencv2/core.hpp>
#include "VirtualCamera.h"
using namespace cv;
using namespace std;
class UnderworldRcns
{
public:
	static void decodeTwoGroupOfImg(const vector<string>& imagelist, VirtualCamera& camera1, VirtualCamera& camera2, const int count, vector<Point3f>& pointcloud_tresh, vector<Vec3f>& color_tresh);
};
#endif