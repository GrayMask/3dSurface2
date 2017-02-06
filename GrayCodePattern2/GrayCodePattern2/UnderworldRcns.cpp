#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>
#include "UnderworldRcns.h"
#include "Path.h"
#include "Const.h"
#include "Tools.h"
#include "VizHelper.h"
#include "VirtualCamera.h"

using namespace cv;
using namespace std;


double matGet2D(Mat m, int x, int y)
{
	int type = m.type();

	switch (type)
	{
	case CV_8U:
		return m.at<uchar>(x, y);
		break;
	case CV_8S:
		return m.at<schar>(x, y);
		break;
	case CV_16U:
		return m.at<ushort>(x, y);
		break;
	case CV_16S:
		return m.at<short>(x, y);
		break;
	case CV_32S:
		return m.at<int>(x, y);
		break;
	case CV_32F:
		return m.at<float>(x, y);
		break;
	case CV_64F:
		return m.at<double>(x, y);
		break;
	}

}

void matSet2D(Mat m, int x, int y, double val)
{
	int type = m.type();

	switch (type)
	{
	case CV_8U:
		m.at<uchar>(x, y) = (uchar)val;
		break;
	case CV_8S:
		m.at<schar>(x, y) = (schar)val;
		break;
	case CV_16U:
		m.at<ushort>(x, y) = (ushort)val;
		break;
	case CV_16S:
		m.at<short>(x, y) = (short)val;
		break;
	case CV_32S:
		m.at<int>(x, y) = (int)val;
		break;
	case CV_32F:
		m.at<float>(x, y) = (float)val;
		break;
	case CV_64F:
		m.at<double>(x, y) = (double)val;
		break;
	}

}

void computeShadows(const Mat& whiteImage, const Mat& blackImage, Mat& shadowMask)
{
	std::cout << "Estimating Shadows...";

	int w = cam_width;
	int h = cam_height;
	shadowMask = Mat(h, w, CV_8U, Scalar(0));

	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{
			float blackVal, whiteVal;
			whiteVal = (float)matGet2D(whiteImage, i, j);
			blackVal = (float)matGet2D(blackImage, i, j);

			if (whiteVal - blackVal > black_thresh)
			{
				matSet2D(shadowMask, i, j, 1);
			}
			else
			{
				matSet2D(shadowMask, i, j, 0);
			}
		}
	}

	std::cout << "done!\n";
}

//access
int ac(int x, int y)
{
	return x*proj_height + y;
}

bool XOR(bool val1, bool val2)
{
	if (val1 == val2)
		return 0;
	else
		return 1;
}

//convert a gray code sequence to a decimal number
int grayToDec(vector<bool> gray)
{
	int dec = 0;
	bool tmp = gray[0];

	if (tmp)
		dec += (int)pow((float)2, int(gray.size() - 1));

	for (int i = 1; i< gray.size(); i++)
	{
		tmp = XOR(tmp, gray[i]);

		if (tmp)
			dec += (int)pow((float)2, int(gray.size() - i - 1));
	}
	return dec;
}

//for a (x,y) pixel of the camera returns the corresponding projector pixel
bool getProjPixel(int x, int y, vector<Mat> &captured_pattern, Point &p_out)
{
	vector<bool> grayCol;
	vector<bool> grayRow;

	bool error = false;
	int xDec, yDec;
	int numOfColBits = 10;
	int numOfRowBits = 10;
	//prosses column images
	for (int count = 0; count<numOfColBits; count++)
	{
		//get pixel intensity for regular pattern projection and it's inverse 
		double val1, val2;
		val1 = matGet2D(captured_pattern[count * 2], x, y);
		val2 = matGet2D(captured_pattern[count * 2 + 1], x, y);

		//check if intensity deference is in a valid rage
		if (abs(val1 - val2) < white_thresh)
			error = true;

		//determine if projection pixel is on or off
		if (val1 > val2)
			grayCol.push_back(1);
		else
			grayCol.push_back(0);

	}
	xDec = grayToDec(grayCol);

	//prosses row images
	for (int count = 0; count < numOfRowBits; count++)
	{

		double val1, val2;

		val1 = matGet2D(captured_pattern[count * 2 + numOfColBits * 2], x, y);
		val2 = matGet2D(captured_pattern[count * 2 + numOfColBits * 2 + 1], x, y);

		if (abs(val1 - val2) < white_thresh)  //check if the difference between the values of the normal and it's inverce projection image is valid
			error = true;

		if (val1 > val2)
			grayRow.push_back(1);
		else
			grayRow.push_back(0);

	}
	yDec = grayToDec(grayRow);

	if ((yDec >= proj_height || xDec >= proj_width))
	{
		error = true;
	}

	p_out.x = xDec;
	p_out.y = yDec;

	return error;
}

void decodePaterns(const vector<Mat>& whiteImages, const vector<Mat>& blackImages, vector<vector<Mat>>& captured_pattern, vector<vector<vector<Point>>>& camsPixels)
{
	std::cout << "Decoding paterns...";
	int numOfCams = camsPixels.size();
	int w = cam_width;
	int h = cam_height;
	for (int c = 0; c < numOfCams; c++) {
		Mat shadowMask;
		computeShadows(whiteImages[c], blackImages[c], shadowMask);
		Point projPixel;
		camsPixels[c].resize(proj_width * proj_height);

		for (int i = 0; i<h; i++)
		{
			for (int j = 0; j<w; j++)
			{
				//if the pixel is not shadow reconstruct
				if (shadowMask.at<uchar>(i, j))
				{

					//get the projector pixel for camera (i,j) pixel
					bool error = getProjPixel(i, j, captured_pattern[c], projPixel);

					if (error)
					{
						shadowMask.at<uchar>(i, j) = 0;
						continue;
					}
					camsPixels[c][ac(projPixel.x, projPixel.y)].push_back(Point(j, i));
				}
			}
		}
	}
	std::cout << "done!\n";
}

Point2f undistortPoints(Point2f p, VirtualCamera cam)
{

	double  k[5] = { 0,0,0,0,0 }, fx, fy, ifx, ify, cx, cy;

	int iters = 1;

	k[0] = cam.distortion.at<float>(0);
	k[1] = cam.distortion.at<float>(1);
	k[2] = cam.distortion.at<float>(2);
	k[3] = cam.distortion.at<float>(3);
	k[4] = 0;

	iters = 5;

	fx = cam.fc.x;
	fy = cam.fc.y;

	ifx = 1. / fx;
	ify = 1. / fy;
	cx = cam.cc.x;
	cy = cam.cc.y;


	double x, y, x0, y0;

	x = p.x;
	y = p.y;

	x0 = x = (x - cx)*ifx;
	y0 = y = (y - cy)*ify;

	for (int jj = 0; jj < iters; jj++)
	{
		double r2 = x*x + y*y;
		double icdist = 1. / (1 + ((k[4] * r2 + k[1])*r2 + k[0])*r2);
		double deltaX = 2 * k[2] * x*y + k[3] * (r2 + 2 * x*x);
		double deltaY = k[2] * (r2 + 2 * y*y) + 2 * k[3] * x*y;
		x = (x0 - deltaX)*icdist;
		y = (y0 - deltaY)*icdist;
	}

	return Point2f((float)(x*fx) + cx, (float)(y*fy) + cy);
}

Point3f pixelToImageSpace(Point2f p, VirtualCamera cam)
{
	Point3f point;

	point.x = (p.x - cam.cc.x) / cam.fc.x;
	point.y = (p.y - cam.cc.y) / cam.fc.y;
	point.z = 1;

	return point;
}

//convert a point from camera to world space
void cam2WorldSpace(VirtualCamera cam, cv::Point3f &p)
{

	cv::Mat tmp(3, 1, CV_32F);
	cv::Mat tmpPoint(3, 1, CV_32F);

	tmpPoint.at<float>(0) = p.x;
	tmpPoint.at<float>(1) = p.y;
	tmpPoint.at<float>(2) = p.z;

	tmp = -cam.rotationMatrix.t() * cam.translationVector;
	tmpPoint = cam.rotationMatrix.t() * tmpPoint;

	p.x = tmp.at<float>(0) + tmpPoint.at<float>(0);
	p.y = tmp.at<float>(1) + tmpPoint.at<float>(1);
	p.z = tmp.at<float>(2) + tmpPoint.at<float>(2);

}

void normalize(cv::Vec3f &vec)
{
	double mag = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

	vec[0] /= (float)max(0.000001, mag);
	vec[1] /= (float)max(0.000001, mag);
	vec[2] /= (float)max(0.000001, mag);

	return;
}

cv::Vec3d matGet3D(cv::Mat m, int x, int y)
{
	int type = m.type();

	switch (type)
	{
	case CV_8U:
	case CV_MAKETYPE(CV_8U, 3):
		return m.at<cv::Vec3b>(x, y);
		break;
	case CV_8S:
	case CV_MAKETYPE(CV_8S, 3):
		return m.at<cv::Vec3b>(x, y);
		break;
	case CV_16U:
	case CV_MAKETYPE(CV_16U, 3):
		return m.at<cv::Vec3w>(x, y);
		break;
	case CV_16S:
	case CV_MAKETYPE(CV_16S, 3):
		return m.at<cv::Vec3s>(x, y);
		break;
	case CV_32S:
	case CV_MAKETYPE(CV_32S, 3):
		return m.at<cv::Vec3i>(x, y);
		break;
	case CV_32F:
	case CV_MAKETYPE(CV_32F, 3):
		return m.at<cv::Vec3f>(x, y);
		break;
	case CV_64F:
	case CV_MAKETYPE(CV_64F, 3):
		return m.at<cv::Vec3d>(x, y);
		break;
	}

}

bool line_lineIntersection(cv::Point3f p1, cv::Vec3f v1, cv::Point3f p2, cv::Vec3f v2, cv::Point3f &p)
{

	cv::Vec3f v12;
	v12 = p1 - p2;

	float v1_dot_v1 = v1.dot(v1);
	float v2_dot_v2 = v2.dot(v2);
	float v1_dot_v2 = v1.dot(v2);
	float v12_dot_v1 = v12.dot(v1);
	float v12_dot_v2 = v12.dot(v2);


	float s, t, denom;


	denom = v1_dot_v1 * v2_dot_v2 - v1_dot_v2 * v1_dot_v2;

	if (abs(denom)<0.001)
		return false;

	s = (v1_dot_v2 / denom) * v12_dot_v2 - (v2_dot_v2 / denom) * v12_dot_v1;
	t = -(v1_dot_v2 / denom) * v12_dot_v1 + (v1_dot_v1 / denom) * v12_dot_v2;

	p = (p1 + s*(cv::Point3f)v1) + (p2 + t*(cv::Point3f) v2);

	p = 0.5*p;

	return true;
}

static void savePointCloud(vector<Point3f>& pointcloud_tresh, vector<Vec3f>& color_tresh, String fileName) {

	ofstream out(root_dir + expr_dir + fileName);
	if (out.is_open())
	{
		int pointCount = pointcloud_tresh.size();
		out << "ply\nformat ascii 1.0\ncomment Kinect v1 generated\nelement vertex " << pointCount << "\nproperty double x\nproperty double y\nproperty double z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
		for (int i = 0; i<pointCount; i++) {
			Point3f point = pointcloud_tresh[i];
			Vec3f color = color_tresh[i];
			out << point.x << " " << point.y << " " << point.z
				<< " " << (int)color[2] << " " << (int)color[1] << " " << (int)color[0] << "\n";
		}
		out.close();
	}
}

void triangulation(vector<Mat> colorImgs, vector<vector<Point>>& cam1Pixels, VirtualCamera& camera1, vector<vector<Point>>& cam2Pixels, VirtualCamera& camera2, int cam1index, int cam2index, vector<Point3f>& pointcloud_tresh, vector<Vec3f>& color_tresh)
{
	pointcloud_tresh.resize(0);
	color_tresh.resize(0);
	int w = proj_width;
	int h = proj_height;
	//start reconstraction
	int load = 0;

	//reconstraction for every projector pixel
	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{

			if (load != (int)(((j + (float)i*h) / ((float)w*h)) * 100))
			{
				load = (int)(((j + (float)i*h) / ((float)w*h)) * 100);
				system("cls");
				std::cout << "Computing 3D Cloud " << load << "%";
			}

			vector<Point> cam1Pixs, cam2Pixs;

			cam1Pixs = cam1Pixels[ac(i, j)];
			cam2Pixs = cam2Pixels[ac(i, j)];

			Point3f reconstructedPoint(0, 0, 0);

			if (cam1Pixs.size() == 0 || cam2Pixs.size() == 0)
				continue;

			Vec3f color1, color2;

			for (int c1 = 0; c1 < cam1Pixs.size(); c1++)
			{

				Point2f camPixelUD = undistortPoints(Point2f(cam1Pixs[c1].x, cam1Pixs[c1].y), camera1);//camera 3d point p for (i,j) pixel
				Point3f cam1Point = pixelToImageSpace(camPixelUD, camera1); //convert camera pixel to image space
				cam2WorldSpace(camera1, cam1Point);

				Vec3f ray1Vector = (Vec3f) (camera1.position - cam1Point); //compute ray vector 
				normalize(ray1Vector);

				//get pixel color for the first camera view
				color1 = matGet3D(colorImgs[cam1index], cam1Pixs[c1].y, cam1Pixs[c1].x);

				for (int c2 = 0; c2 < cam2Pixs.size(); c2++)
				{

					camPixelUD = undistortPoints(Point2f(cam2Pixs[c2].x, cam2Pixs[c2].y), camera2);//camera 3d point p for (i,j) pixel

					Point3f cam2Point = pixelToImageSpace(camPixelUD, camera2); //convert camera pixel to image space
					cam2WorldSpace(camera2, cam2Point);

					Vec3f ray2Vector = (Vec3f) (camera2.position - cam2Point); //compute ray vector 
					normalize(ray2Vector);

					Point3f interPoint;

					bool ok = line_lineIntersection(camera1.position, ray1Vector, camera2.position, ray2Vector, interPoint);


					if (!ok)
						continue;

					//get pixel color for the second camera view
					color2 = matGet3D(colorImgs[cam2index], cam2Pixs[c2].y, cam2Pixs[c2].x);
					pointcloud_tresh.push_back(interPoint);
					color_tresh.push_back((color1 + color2) / 2);
				}
			}
		}
	}


	system("cls");
	std::cout << "Computing 3D Cloud  100%\n";

}

void UnderworldRcns::decodeTwoGroupOfImg(const vector<string>& imagelist, VirtualCamera& camera1, VirtualCamera& camera2, const int count, vector<Point3f>& pointcloud_tresh, vector<Vec3f>& color_tresh) {
	int numberOfPatternImages = 40;
	vector<vector<Mat>> captured_pattern;
	captured_pattern.resize(2);
	captured_pattern[0].resize(numberOfPatternImages);
	captured_pattern[1].resize(numberOfPatternImages);
	vector<Mat> color;
	color.resize(2);
	color[0] = imread(imagelist[numberOfPatternImages], IMREAD_COLOR);
	color[1] = imread(imagelist[2 * numberOfPatternImages + 2], IMREAD_COLOR);
	for (size_t i = 0; i < numberOfPatternImages; i++)
	{
		captured_pattern[0][i] = imread(imagelist[i], IMREAD_GRAYSCALE);
		captured_pattern[1][i] = imread(imagelist[i + numberOfPatternImages + 2], IMREAD_GRAYSCALE);
		if ((!captured_pattern[0][i].data) || (!captured_pattern[1][i].data))
		{
			cout << "Empty images" << endl;
			return;
		}
	}
	vector<Mat> whiteImages;
	vector<Mat> blackImages;
	whiteImages.resize(2);
	blackImages.resize(2);
	cvtColor(color[0], whiteImages[0], COLOR_RGB2GRAY);
	cvtColor(color[1], whiteImages[1], COLOR_RGB2GRAY);
	blackImages[0] = imread(imagelist[numberOfPatternImages + 1], IMREAD_GRAYSCALE);
	blackImages[1] = imread(imagelist[2 * numberOfPatternImages + 2 + 1], IMREAD_GRAYSCALE);
	vector<vector<vector<Point>>> camsPixels;
	camsPixels.resize(2);
	decodePaterns(whiteImages, blackImages, captured_pattern, camsPixels);

	camera1.position = cv::Point3f(0, 0, 0);
	cam2WorldSpace(camera1, camera1.position);
	camera2.position = cv::Point3f(0, 0, 0);
	cam2WorldSpace(camera2, camera2.position);
	triangulation(color, camsPixels[0], camera1, camsPixels[1], camera2, 0, 1, pointcloud_tresh, color_tresh);
	ostringstream countStr;
	//countStr << i;
	countStr << count;
	savePointCloud(pointcloud_tresh, color_tresh, countStr.str() + ply_file);
}
