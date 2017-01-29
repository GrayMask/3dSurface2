#include <opencv2/core.hpp>;
#include <opencv2/viz.hpp>
#include "VizHelper.h"

using namespace cv;
using namespace std;


void VizHelper::showPointCloud(InputArray pointcloud, InputArray color) {
	vector<Mat> pointcloudArr;
	vector<Mat> colorArr;
	if (pointcloud.kind() == _InputArray::MAT) {
		pointcloudArr.resize(0);
		pointcloudArr.push_back(pointcloud.getMat());
		colorArr.resize(0);
		colorArr.push_back(color.getMat());
	}
	else {
		pointcloud.getMatVector(pointcloudArr);
		color.getMatVector(colorArr);
	}
	viz::Viz3d myWindow("Point cloud with color");
	myWindow.setBackgroundMeshLab();
	myWindow.showWidget("coosys", viz::WCoordinateSystem());
	int length = pointcloudArr.size();
	for (int i = 0; i < length; i++) {
		stringstream ss;
		string s;
		ss << "pointcloud" << i;
		ss >> s;
		myWindow.showWidget(s, viz::WCloud(pointcloudArr[i], colorArr[i]));
	}
	myWindow.showWidget("text2d", viz::WText("Point cloud", Point(20, 20), 20, viz::Color::green()));
	myWindow.spin();
}