 #include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/structured_light.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv_modules.hpp>
#include <iostream>
#include <stdio.h>
#include "GrayCodePattern.h"
#include "Const.h"
#include "Path.h"
// (if you did not build the opencv_viz module, you will only see the disparity images)

#include <opencv2/viz.hpp>

using namespace cv;
using namespace std;

#define cvQueryHistValue_1D( hist, idx0 ) \
    ((float)cvGetReal1D( (hist)->bins, (idx0)))

int abWrite(const Mat &im, const string &fname)
{
	ofstream ouF;
	ouF.open(fname.c_str(), std::ofstream::binary);
	if (!ouF)
	{
		cerr << "failed to open the file : " << fname << endl;
		return 0;
	}
	for (int r = 0; r < im.rows; r++)
	{
		ouF.write(reinterpret_cast<const char*>(im.ptr(r)), im.cols*im.elemSize());
	}
	ouF.close();
	return 1;
}

void GrayCodePattern::getGrayCodeImages()
{
	structured_light::GrayCodePattern::Params params;
	params.width = proj_width;
	params.height = proj_height;
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	// Storage for pattern
	vector<Mat> pattern;
	graycode->generate(pattern);
	cout << pattern.size() << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
		<< endl;
	// Generate the all-white and all-black images needed for shadows mask computation
	Mat white;
	Mat black;
	graycode->getImagesForShadowMasks(black, white);
	pattern.push_back(white);
	pattern.push_back(black);
	// Setting pattern window on second monitor (the projector's one)
	namedWindow("Pattern Window", WINDOW_NORMAL);
	resizeWindow("Pattern Window", params.width, params.height);
	moveWindow("Pattern Window", params.width + 316, -20);
	setWindowProperty("Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	// Open camera number 1, using libgphoto2
	VideoCapture cap1(1);
	if (!cap1.isOpened())
	{
		// check if cam1 opened
		cout << "cam1 not opened!" << endl;
		exit(-1);
	}
	// Turning off autofocus
	cap1.set(CAP_PROP_SETTINGS, 1);
	int i = 0;
	while (i < (int)pattern.size())
	{
		cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
		imshow("Pattern Window", pattern[i]);
		Mat frame1;
		while (1)
		{
			cap1 >> frame1;  // get a new frame from camera 1
			imshow("cam1", frame1);
			int key = waitKey(1);
			if (key == 115)
			{
				bool save1 = false;
				ostringstream name;
				name << i + 1;
				save1 = imwrite(images_dir + "pattern_cam1_im" + name.str() + imgType, frame1);
				if (save1)
				{
					cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
					i++;
					break;
				}
				else
				{
					cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path" << endl << endl;
				}
			}
		}
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "failed to open " << filename << endl;
		return false;
	}
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
	{
		cerr << "cam 1 images are not a sequence! FAIL" << endl;
		return false;
	}
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
	{
		l.push_back((string)*it);
	}
	n = fs["cam2"];
	if (n.type() != FileNode::SEQ)
	{
		cerr << "cam 2 images are not a sequence! FAIL" << endl;
		return false;
	}
	it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
	{
		l.push_back((string)*it);
	}
	if (l.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
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

	for (int i = 0; i<hist_size; i++)
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

bool sort_by_value(const uchar& obj1, const uchar& obj2)
{
	return obj1 < obj2;
}

static int optimizeDisparityMap(const Mat disparityMap, Mat& result)
{
	// Find the mid value
	int invalid = 0;// the invalid value
	int length = disparityMap.rows * disparityMap.cols;
	vector<uchar> array(disparityMap.data, disparityMap.data + length);
	vector<uchar>::iterator it;
	for (it = array.begin(); it != array.end();)
	{
		if (*it == invalid)
			it = array.erase(it);
		else
			++it;
	}
	sort(array.begin(), array.end(), sort_by_value);
	float numOfValid = array.size();
	uchar max_value = array[numOfValid - 1];
	uchar upThresh = array[numOfValid * upPortion];
	uchar downThresh = array[numOfValid * downPortion];

	// Use threshold to filter the disparityMap
	threshold(disparityMap, result, upThresh, max_value, THRESH_TOZERO_INV);
	threshold(result, result, downThresh, max_value, THRESH_TOZERO);
	return downThresh;
}

int GrayCodePattern::executeDecode()
{
	structured_light::GrayCodePattern::Params params;
	params.width = proj_width;
	params.height = proj_height;
	// Set up GraycodePattern with params
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	if (isThresh) {
		graycode->setWhiteThreshold(white_thresh);
		graycode->setBlackThreshold(black_thresh);
	}
	vector<string> imagelist;
	bool ok = readStringList(imagesName_file, imagelist);
	if (!ok || imagelist.empty())
	{
		cout << "can not open " << imagesName_file << " or the string list is empty" << endl;
		return -1;
	}
	FileStorage fs(calib_file, FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "Failed to open Calibration Data File." << endl;
		return -1;
	}
	// Loading calibration parameters
	Mat cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, R, T;
	fs["cam1_intrinsics"] >> cam1intrinsics;
	fs["cam2_intrinsics"] >> cam2intrinsics;
	fs["cam1_distorsion"] >> cam1distCoeffs;
	fs["cam2_distorsion"] >> cam2distCoeffs;
	fs["R"] >> R;
	fs["T"] >> T;
	cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
	cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;
	cout << "cam2intrinsics" << endl << cam2intrinsics << endl;
	cout << "cam2distCoeffs" << endl << cam2distCoeffs << endl;
	cout << "T" << endl << T << endl << "R" << endl << R << endl;
	if ((!R.data) || (!T.data) || (!cam1intrinsics.data) || (!cam2intrinsics.data) || (!cam1distCoeffs.data) || (!cam2distCoeffs.data))
	{
		cout << "Failed to load cameras calibration parameters" << endl;
		return -1;
	}
	size_t numberOfPatternImages = graycode->getNumberOfPatternImages();
	vector<vector<Mat> > captured_pattern;
	captured_pattern.resize(2);
	captured_pattern[0].resize(numberOfPatternImages);
	captured_pattern[1].resize(numberOfPatternImages);
	Mat color = imread(imagelist[numberOfPatternImages], IMREAD_COLOR);
	Size imagesSize = color.size();
	// Stereo rectify
	cout << "Rectifying images..." << endl;
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];
	stereoRectify(cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, imagesSize, R, T, R1, R2, P1, P2, Q, 0,
		-1, imagesSize, &validRoi[0], &validRoi[1]);
	Mat map1x, map1y, map2x, map2y;
	initUndistortRectifyMap(cam1intrinsics, cam1distCoeffs, R1, P1, imagesSize, CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(cam2intrinsics, cam2distCoeffs, R2, P2, imagesSize, CV_32FC1, map2x, map2y);
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
		myCalcHist(scaledDisparityMap);

		abWrite(scaledDisparityMap, disparityMap_file);

		applyColorMap(scaledDisparityMap, cm_disp, COLORMAP_RAINBOW);
		// Show the result
		resize(cm_disp, cm_disp, Size(640, 480));
		imshow("cm disparity m", cm_disp);
		imwrite("cv_disparity_m.png", cm_disp);
		// Compute the point cloud
		Mat pointcloud;
		disparityMap.convertTo(disparityMap, CV_32FC1);
		reprojectImageTo3D(disparityMap, pointcloud, Q, false, -1);
		// Compute a mask to remove background
		Mat dst, thresholded_disp;
		threshold(scaledDisparityMap, thresholded_disp, 0, 255, THRESH_OTSU + THRESH_BINARY);
		resize(thresholded_disp, dst, Size(640, 480));
		imshow("threshold disp otsu", scaledDisparityMap);
		imwrite("threshold_disp_otsu.png", scaledDisparityMap);

		// Apply the mask to the point cloud
		Mat pointcloud_tresh, color_tresh;
		pointcloud.copyTo(pointcloud_tresh, thresholded_disp);
		color.copyTo(color_tresh, thresholded_disp);
		minMaxIdx(color_tresh, &min, &max);
		cout << "color_tresh min " << min << endl << "disp max " << max << endl;
		minMaxIdx(pointcloud_tresh, &min, &max);
		cout << "pointcloud_tresh min " << min << endl << "disp max " << max << endl;
		// Show the point cloud on viz
		viz::Viz3d myWindow("Point cloud with color");
		myWindow.setBackgroundMeshLab();
		myWindow.showWidget("coosys", viz::WCoordinateSystem());
		myWindow.showWidget("pointcloud", viz::WCloud(pointcloud_tresh, color_tresh));
		myWindow.showWidget("text2d", viz::WText("Point cloud", Point(20, 20), 20, viz::Color::green()));
		myWindow.spin();

	}
	waitKey();
	return 0;
}
