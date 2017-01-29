#include <opencv2/core.hpp>;
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
#ifndef VIRTUALCAMERA_H
#define VIRTUALCAMERA_H
class VirtualCamera
{

public:

	VirtualCamera(void);
	VirtualCamera(Mat distortion_, Mat rotationMatrix_, Mat translationVector_, Mat cameraMatrix_);
	~VirtualCamera(void);


	Mat distortion;
	Mat rotationMatrix;
	Mat translationVector;
	Mat cameraMatrix;

	Point3f position;

	Point2f fc;
	Point2f cc;

	//int width;
	//int height;

};
#endif