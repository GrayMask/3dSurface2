#include <opencv2/structured_light.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include "Decode.h"
#include "Path.h"
#include "Const.h"
#include "Tools.h"
#include "OpencvRcns.h"
#include "VizHelper.h"
#include "UnderworldRcns.h"

using namespace cv;
using namespace std;

static bool readImageList(const int count1, const int count2, vector<string>& l)
{
	l.resize(0);
	char* imagesGroupDirTemp = new char[images_group_dir_length];
	sprintf(imagesGroupDirTemp, images_group_dir, count1);
	String filename = root_dir + expr_dir + String(imagesGroupDirTemp) + imagesName_file;
	if (Tools::readStringList(filename, l) == -1) {
		return false;
	}
	sprintf(imagesGroupDirTemp, images_group_dir, count2);
	filename = root_dir + expr_dir + String(imagesGroupDirTemp) + imagesName_file;
	if (Tools::readStringList(filename, l) == -1) {
		return false;
	}
	return true;
}

static void getRAndTBetweenTwoCamera(const Mat& R1, const Mat& T1, const Mat& R2, const Mat& T2, Mat& R, Mat& T) {
	Mat R1T;
	transpose(R1, R1T);
	R = R2 * R1T;
	T = T2 - R * T1;
}

static void transformPointCloud(const Mat& R, const Mat& T, Mat& pointcloud) {
	Mat R_, T_, RT;
	R.convertTo(R_, CV_32FC1);
	T.convertTo(T_, CV_32FC1);
	transpose(R_, RT);
	//float matrix[3][3] = { {-1, 0, 0},{ 0, -1, 0},{ 0, 0, 1} };
	//Mat A(Size(3, 3), CV_32FC1, matrix);
	Size sz = pointcloud.size();
	for (int i = 0; i < sz.height; i++) {
		for (int j = 0; j < sz.width; j++) {
			Vec3f x = pointcloud.at<Vec3f>(i, j);
			Mat ym = RT * (Mat(x) - T_);
			Vec3f y = (Vec3f) Mat(ym);
			pointcloud.at<Vec3f>(i, j) = y;
		}
	}
}

struct Point3dRGB {
	cv::Point3d point;
	uchar r;
	uchar g;
	uchar b;
};

static void savePointCloud(InputArray pointcloud, InputArray color, String fileName) {
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
	vector<Point3dRGB> pointcloudList;
	pointcloudList.resize(0);
	int length = pointcloudArr.size();
	for (int i = 0; i < length; i++) {
		Mat pointcloudMat = pointcloudArr[i];
		Mat colorMat = colorArr[i];
		Size sz = pointcloudMat.size();
		for (int x = 0; x < sz.height; x++) {
			for (int y = 0; y < sz.width; y++) {
				Vec3f point = pointcloudMat.at<Vec3f>(x, y);
				if (point[0] != 0 || point[1] != 0 || point[2] != 0) {
					Point3dRGB point3dRGB;
					point3dRGB.point.x = point[0];
					point3dRGB.point.y = point[1];
					point3dRGB.point.z = point[2];
					point3dRGB.r = colorMat.at<Vec3b>(x, y)[2];
					point3dRGB.g = colorMat.at<Vec3b>(x, y)[1];
					point3dRGB.b = colorMat.at<Vec3b>(x, y)[0];
					pointcloudList.push_back(point3dRGB);
				}
			}
		}
	}
	ofstream out(root_dir + expr_dir + fileName);
	if (out.is_open())
	{
		int pointCount = pointcloudList.size();
		out << "ply\nformat ascii 1.0\ncomment Kinect v1 generated\nelement vertex " << pointCount << "\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
		std::vector<Point3dRGB>::iterator begin;
		std::vector<Point3dRGB>::iterator end;
		begin = pointcloudList.begin();
		end = pointcloudList.end();
		for (; begin != end; ++begin) {
			out << begin->point.x << " " << begin->point.y << " " << begin->point.z
				<< " " << (int)begin->r << " " << (int)begin->g << " " << (int)begin->b << "\n";
		}
		out.close();
	}
}

int Decode::executeDecode() {
	structured_light::GrayCodePattern::Params params;
	params.width = proj_width;
	params.height = proj_height;
	// Set up GraycodePattern with params
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	if (isThresh) {
		graycode->setWhiteThreshold(white_thresh);
		graycode->setBlackThreshold(black_thresh);
	}
	FileStorage fs(root_dir + calib_file, FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "Failed to open Calibration Data File." << endl;
		return -1;
	}
	// Loading calibration parameters
	Mat intrinsics, distCoeffs;
	fs["intrinsics"] >> intrinsics;
	fs["distorsion"] >> distCoeffs;
	if ((!intrinsics.data) || (!distCoeffs.data))
	{
		cout << "Failed to load cameras calibration parameters" << endl;
		return -1;
	}
	int groupNum;
	Tools::readGroupNumFile(groupNum);
	vector<Mat> pointcloudArr;
	pointcloudArr.resize(0);
	vector<Mat> colorArr;
	colorArr.resize(0);
	for (int i = 0; i < groupNum - 1; i++) {
		Mat R, T, R1, T1, R2, T2;
		if (Tools::getSFMResult(i, R1, T1)) {
			int j;
			for (j = i + 1; j < groupNum; j++) {
				if (Tools::getSFMResult(j, R2, T2)) {
					vector<string> imagelist;
					bool ok = readImageList(i, j, imagelist);
					if (!ok || imagelist.empty())
					{
						cout << "can not open " << imagesName_file << " or the string list is empty" << endl;
						return -1;
					}
					Mat pointcloud_tresh, color_tresh;
					if (isUnderWorld) {
						VirtualCamera camera1(intrinsics, distCoeffs, R1, T1);
						VirtualCamera camera2(intrinsics, distCoeffs, R2, T2);
						vector<Point3f> pointclouds;
						vector<Vec3f> colors;
						UnderworldRcns::decodeTwoGroupOfImg(imagelist, camera1, camera2, i, pointclouds, colors);
					}
					else {
						getRAndTBetweenTwoCamera(R2, T2, R1, T1, R, T);
						cout << "Analyzing " << i << " and " << j << endl;
						cout << R << endl;
						cout << T << endl;
						OpencvRcns::decodeTwoGroupOfImg(graycode, imagelist, intrinsics, distCoeffs, R, T, i, pointcloud_tresh, color_tresh);
					}
					transformPointCloud(R1, T1, pointcloud_tresh);
					ostringstream countStrI;
					ostringstream countStrJ;
					countStrI << i;
					countStrJ << j;
					savePointCloud(pointcloud_tresh, color_tresh, countStrI.str() + countStrJ.str() + ply_file);
					pointcloudArr.push_back(pointcloud_tresh);
					colorArr.push_back(color_tresh);
					//break;
				}
			}
			break;
		}
	}
	//savePointCloud(pointcloudArr, colorArr, ply_file);
	VizHelper::showPointCloud(pointcloudArr, colorArr);
}
