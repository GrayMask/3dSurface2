#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>
#include "Decode.h"
#include "Path.h"
#include "Const.h"
#include "Tools.h"

using namespace cv;
using namespace std;

#define cvQueryHistValue_1D( hist, idx0 ) \
    ((float)cvGetReal1D( (hist)->bins, (idx0)))

static bool readImageList(const int count1, const int count2, vector<string>& l)
{
	l.resize(0);
	char* imagesGroupDirTemp = new char[images_group_dir_length];
	sprintf(imagesGroupDirTemp, images_group_dir, count2);
	String filename = root_dir + expr_dir + String(imagesGroupDirTemp) + imagesName_file;
	if (Tools::readStringList(filename, l) == -1) {
		return false;
	}
	sprintf(imagesGroupDirTemp, images_group_dir, count1);
	filename = root_dir + expr_dir + String(imagesGroupDirTemp) + imagesName_file;
	if (Tools::readStringList(filename, l) == -1) {
		return false;
	}
	return true;
}

static void myCalcHist(Mat gray_plane)
{
	IplImage *src;
	src = &IplImage(gray_plane);
	int hist_size = 256;
	int hist_height = 256;
	float range[] = { 0,255 };
	float* ranges[] = { range };
	CvHistogram* gray_hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
	cvCalcHist(&src, gray_hist, 0, 0);
	cvNormalizeHist(gray_hist, 1.0);

	int scale = 2;
	IplImage* hist_image = cvCreateImage(cvSize(hist_size*scale, hist_height), 8, 3);
	cvZero(hist_image);
	float max_value = 0;
	cvGetMinMaxHistValue(gray_hist, 0, &max_value, 0, 0);

	for (int i = 0; i < hist_size; i++)
	{
		float bin_val = cvQueryHistValue_1D(gray_hist, i);
		int intensity = cvRound(bin_val*hist_height / max_value);
		cvRectangle(hist_image,
			cvPoint(i*scale, hist_height - 1),
			cvPoint((i + 1)*scale - 1, hist_height - intensity),
			CV_RGB(255, 255, 255));
	}
	cvGetMinMaxHistValue(gray_hist, 0, &max_value, 0, 0);
	cvNamedWindow("H-S Histogram", 1);
	cvShowImage("H-S Histogram", hist_image);
}

bool sort_by_value(const float& obj1, const float& obj2)
{
	return obj1 < obj2;
}

static int optimizeDisparityMap(const Mat disparityMap, Mat& result)
{
	// Find the mid value
	float invalid = 0;// the invalid value
	std::vector<float> array;
	array.assign((float*)disparityMap.datastart, (float*)disparityMap.dataend);
	vector<float>::iterator it;
	for (it = array.begin(); it != array.end();)
	{
		if (*it == invalid)
			it = array.erase(it);
		else
			++it;
	}
	sort(array.begin(), array.end(), sort_by_value);
	float numOfValid = array.size();
	float max_value = array[numOfValid - 1];
	float upThresh = array[numOfValid * upPortion];
	float downThresh = array[numOfValid * downPortion];

	// Use threshold to filter the disparityMap
	threshold(disparityMap, result, upThresh, max_value, THRESH_TOZERO_INV);
	threshold(result, result, downThresh, max_value, THRESH_TOZERO);
	return downThresh;
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

static void showPointCloud(InputArray pointcloud, InputArray color) {
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

struct Point3dRGB {
	cv::Point3d point;
	uchar r;
	uchar g;
	uchar b;
};

static void savePointCloud(InputArray pointcloud, InputArray color) {
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
	ofstream out(root_dir + ply_file);
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

static int decodeTwoGroupOfImg(const Ptr<structured_light::GrayCodePattern>& graycode, const vector<string>& imagelist, const Mat& intrinsics, const Mat& distCoeffs, const Mat& R, const Mat& T, const int count, Mat& pointcloud_tresh, Mat& color_tresh)
{
	size_t numberOfPatternImages = graycode->getNumberOfPatternImages();
	vector<vector<Mat>> captured_pattern;
	captured_pattern.resize(2);
	captured_pattern[0].resize(numberOfPatternImages);
	captured_pattern[1].resize(numberOfPatternImages);
	Mat color = imread(imagelist[numberOfPatternImages], IMREAD_COLOR);
	Size imagesSize = color.size();
	// Stereo rectify
	cout << "Rectifying images..." << endl;
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(intrinsics, distCoeffs, intrinsics, distCoeffs, imagesSize, R, T, R1, R2, P1, P2, Q, 0,
		-1, imagesSize, &validRoi[0], &validRoi[1]);
	Mat map1x, map1y, map2x, map2y;
	initUndistortRectifyMap(intrinsics, distCoeffs, R1, P1, imagesSize, CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(intrinsics, distCoeffs, R2, P2, imagesSize, CV_32FC1, map2x, map2y);
	// Loading pattern images
	for (size_t i = 0; i < numberOfPatternImages; i++)
	{
		captured_pattern[0][i] = imread(imagelist[i], IMREAD_GRAYSCALE);
		captured_pattern[1][i] = imread(imagelist[i + numberOfPatternImages + 2], IMREAD_GRAYSCALE);
		if ((!captured_pattern[0][i].data) || (!captured_pattern[1][i].data))
		{
			cout << "Empty images" << endl;
			return -1;
		}
		if (isRemap) {
			remap(captured_pattern[1][i], captured_pattern[1][i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
			remap(captured_pattern[0][i], captured_pattern[0][i], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
		}
	}
	cout << "done" << endl;
	vector<Mat> blackImages;
	vector<Mat> whiteImages;
	blackImages.resize(2);
	whiteImages.resize(2);
	// Loading images (all white + all black) needed for shadows computation
	cvtColor(color, whiteImages[0], COLOR_RGB2GRAY);
	whiteImages[1] = imread(imagelist[2 * numberOfPatternImages + 2], IMREAD_GRAYSCALE);
	blackImages[0] = imread(imagelist[numberOfPatternImages + 1], IMREAD_GRAYSCALE);
	blackImages[1] = imread(imagelist[2 * numberOfPatternImages + 2 + 1], IMREAD_GRAYSCALE);
	if (isRemap) {
		remap(color, color, map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
		remap(whiteImages[0], whiteImages[0], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
		remap(whiteImages[1], whiteImages[1], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
		remap(blackImages[0], blackImages[0], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
		remap(blackImages[1], blackImages[1], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
	}
	cout << endl << "Decoding pattern ..." << endl;
	Mat disparityMap;
	bool decoded = graycode->decode(captured_pattern, disparityMap, blackImages, whiteImages,
		structured_light::DECODE_3D_UNDERWORLD);
	disparityMap.convertTo(disparityMap, CV_32F);
	if (isOptimize) {
		int downThresh = optimizeDisparityMap(disparityMap, disparityMap);
	}
	ostringstream countStr;
	countStr << count;
	Tools::writePic(disparityMap, root_dir + disparityMap_file + countStr.str() + ".txt");
	if (decoded)
	{
		cout << endl << "pattern decoded" << endl;
		// To better visualize the result, apply a colormap to the computed disparity
		double min;
		double max;
		minMaxIdx(disparityMap, &min, &max);
		Mat cm_disp, scaledDisparityMap;
		cout << "disp min " << min << endl << "disp max " << max << endl;
		convertScaleAbs(disparityMap, scaledDisparityMap, 255 / (max - min), -min * 255 / (max - min));

		applyColorMap(scaledDisparityMap, cm_disp, COLORMAP_RAINBOW);
		// Compute the point cloud
		Mat pointcloud;
		reprojectImageTo3D(disparityMap, pointcloud, Q, false, -1);
		// Compute a mask to remove background
		Mat thresholded_disp;
		threshold(abs(disparityMap), thresholded_disp, 0, 255, THRESH_BINARY);
		thresholded_disp.convertTo(thresholded_disp, CV_8U);

		// Apply the mask to the point cloud
		cout << pointcloud.channels() << endl;
		pointcloud.copyTo(pointcloud_tresh, thresholded_disp);
		color.copyTo(color_tresh, thresholded_disp);
		minMaxIdx(color_tresh, &min, &max);
		cout << "color_tresh min " << min << endl << "disp max " << max << endl;
		minMaxIdx(pointcloud_tresh, &min, &max);
		cout << "pointcloud_tresh min " << min << endl << "disp max " << max << endl;
		// Show the point cloud on viz
		if (isShowResult) {
			// Show the result
			myCalcHist(scaledDisparityMap);
			imshow("cm disparity m", cm_disp);
			imshow("threshold disp otsu", scaledDisparityMap);
			showPointCloud(pointcloud_tresh, color_tresh);
			waitKey();
		}
	}
	return 1;
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
					getRAndTBetweenTwoCamera(R1, T1, R2, T2, R, T);
					cout << "Analyzing " << i << " and " << j << endl;
					cout << R << endl;
					cout << T << endl;
					vector<string> imagelist;
					bool ok = readImageList(i, j, imagelist);
					if (!ok || imagelist.empty())
					{
						cout << "can not open " << imagesName_file << " or the string list is empty" << endl;
						return -1;
					}
					Mat pointcloud_tresh, color_tresh;
					decodeTwoGroupOfImg(graycode, imagelist, intrinsics, distCoeffs, R, T, i, pointcloud_tresh, color_tresh);
					transformPointCloud(R2, T2, pointcloud_tresh);
					pointcloudArr.push_back(pointcloud_tresh);
					colorArr.push_back(color_tresh);
					break;
				}
			}
		}
	}
	savePointCloud(pointcloudArr, colorArr);
	showPointCloud(pointcloudArr, colorArr);
}
