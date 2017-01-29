#include <opencv2/core.hpp>;
using namespace cv;
using namespace std;
#ifndef VIZHELPER_H
#define VIZHELPER_H
class VizHelper
{
public:
	static void showPointCloud(InputArray pointcloud, InputArray color);
};
#endif