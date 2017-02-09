#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include "OpencvRcns.h"
#include "Path.h"
#include "Const.h"
#include "Tools.h"
#include "VirtualCamera.h"
#include "VizHelper.h"

using namespace cv;
using namespace std;


bool sort_by_value(const float& obj1, const float& obj2)
{
	return obj1 < obj2;
}

/* Camera 1 is origin*/
static void getRAndTBetweenTwoCamera(const Mat& R1, const Mat& T1, const Mat& R2, const Mat& T2, Mat& R, Mat& T) {
	Mat R2T;
	transpose(R2, R2T);
	R = R1 * R2T;
	T = T1 - R * T2;
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

static void modifyCameraCoord(const Mat& R, Mat& pointcloud) {
	Mat R_, RT;
	R.convertTo(R_, CV_32FC1);
	transpose(R_, RT);
	Size sz = pointcloud.size();
	for (int i = 0; i < sz.height; i++) {
		for (int j = 0; j < sz.width; j++) {
			Vec3f x = pointcloud.at<Vec3f>(i, j);
			Mat ym = RT * Mat(x);
			Vec3f y = (Vec3f)Mat(ym);
			pointcloud.at<Vec3f>(i, j) = y;
		}
	}
}

void OpencvRcns::decodeTwoGroupOfImg(const Ptr<structured_light::GrayCodePattern>& graycode, const vector<string>& imagelist, VirtualCamera& camera1, VirtualCamera& camera2, const int count, Mat& pointcloud_tresh, Mat& color_tresh) {
		size_t numberOfPatternImages = graycode->getNumberOfPatternImages();
		vector<vector<Mat>> captured_pattern;
		captured_pattern.resize(2);
		captured_pattern[0].resize(numberOfPatternImages);
		captured_pattern[1].resize(numberOfPatternImages);
		Mat color = imread(imagelist[numberOfPatternImages], IMREAD_COLOR);
		Size imagesSize = color.size();
		// Stereo rectify
		cout << "Rectifying images..." << endl;
		Mat R1, R2, P1, P2, Q, R, T, I1, I2, D1, D2;
		Rect validRoi[2];
		getRAndTBetweenTwoCamera(camera1.rotationMatrix, camera1.translationVector, camera2.rotationMatrix, camera2.translationVector, R, T);
		cout << R << endl;
		cout << T << endl;
		R.convertTo(R, CV_64F);
		T.convertTo(T, CV_64F);
		camera1.cameraMatrix.convertTo(I1, CV_64F); 
		camera1.distortion.convertTo(D1, CV_64F); 
		camera2.cameraMatrix.convertTo(I2, CV_64F);
		camera2.distortion.convertTo(D2, CV_64F);
		stereoRectify(I1, D1, I2, D2, imagesSize, R, T, R1, R2, P1, P2, Q, 0,
			-1, imagesSize, &validRoi[0], &validRoi[1]);
		Mat map1x, map1y, map2x, map2y;
		initUndistortRectifyMap(I1, D1, R1, P1, imagesSize, CV_32FC1, map1x, map1y);
		initUndistortRectifyMap(I2, D2, R2, P2, imagesSize, CV_32FC1, map2x, map2y);
		// Loading pattern images
		for (size_t i = 0; i < numberOfPatternImages; i++)
		{
			captured_pattern[0][i] = imread(imagelist[i], IMREAD_GRAYSCALE);
			captured_pattern[1][i] = imread(imagelist[i + numberOfPatternImages + 2], IMREAD_GRAYSCALE);
			if ((!captured_pattern[0][i].data) || (!captured_pattern[1][i].data))
			{
				cout << "Empty images" << endl;
				return;
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
			pointcloud.copyTo(pointcloud_tresh, thresholded_disp);
			modifyCameraCoord(R1, pointcloud_tresh);
			cout << R1 << endl;
			color.copyTo(color_tresh, thresholded_disp);
			minMaxIdx(color_tresh, &min, &max);
			cout << "color_tresh min " << min << endl << "disp max " << max << endl;
			minMaxIdx(pointcloud_tresh, &min, &max);
			cout << "pointcloud_tresh min " << min << endl << "disp max " << max << endl;
			// Show the point cloud on viz
			if (isShowResult) {
				// Show the result
				Tools::myCalcHist(scaledDisparityMap);
				imshow("cm disparity m", cm_disp);
				imshow("threshold disp otsu", scaledDisparityMap);
				VizHelper::showPointCloud(pointcloud_tresh, color_tresh);
				waitKey();
			}
		}
}