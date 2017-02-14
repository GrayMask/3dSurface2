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
#include "Utilities.h"
#include "VizHelper.h"
#include "VirtualCamera.h"

using namespace cv;
using namespace std;

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

void triangulation(vector<Mat> colorImgs, vector<vector<Point>>& cam1Pixels, VirtualCamera& camera1, vector<vector<Point>>& cam2Pixels, VirtualCamera& camera2, int cam1index, int cam2index, vector<Point3f>& pointcloud_tresh, vector<Vec3f>& color_tresh)
{
	int w = proj_width;
	int h = proj_height;
	//start reconstraction
	int load = 0;

	//reconstraction for every projector pixel
	for (int i = 0; i<w; i++)
	{
		for (int j = 0; j<h; j++)
		{

			if (load != (int)(((j + (float)i*h) / ((float)w*h)) * 100))
			{
				load = (int)(((j + (float)i*h) / ((float)w*h)) * 100);
				system("cls");
				std::cout << "Computing 3D Cloud " << load << "%";
			}

			vector<cv::Point> cam1Pixs, cam2Pixs;

			cam1Pixs = cam1Pixels[Utilities::ac(i, j)];
			cam2Pixs = cam2Pixels[Utilities::ac(i, j)];

			cv::Point3f reconstructedPoint(0, 0, 0);

			if (cam1Pixs.size() == 0 || cam2Pixs.size() == 0)
				continue;

			cv::Vec3f color1, color2;

			for (int c1 = 0; c1 < cam1Pixs.size(); c1++)
			{

				cv::Point2f camPixelUD = Utilities::undistortPoints(cv::Point2f(cam1Pixs[c1].x, cam1Pixs[c1].y), camera1);//camera 3d point p for (i,j) pixel
				cv::Point3f cam1Point = Utilities::pixelToImageSpace(camPixelUD, camera1); //convert camera pixel to image space
				cam2WorldSpace(camera1, cam1Point);

				cv::Vec3f ray1Vector = (cv::Vec3f) (camera1.position - cam1Point); //compute ray vector 
				Utilities::normalize(ray1Vector);

				//get pixel color for the first camera view
				color1 = Utilities::matGet3D(colorImgs[cam1index], cam1Pixs[c1].x, cam1Pixs[c1].y);

				for (int c2 = 0; c2 < cam2Pixs.size(); c2++)
				{

					camPixelUD = Utilities::undistortPoints(cv::Point2f(cam2Pixs[c2].x, cam2Pixs[c2].y), camera2);//camera 3d point p for (i,j) pixel

					cv::Point3f cam2Point = Utilities::pixelToImageSpace(camPixelUD, camera2); //convert camera pixel to image space
					cam2WorldSpace(camera2, cam2Point);

					cv::Vec3f ray2Vector = (cv::Vec3f) (camera2.position - cam2Point); //compute ray vector 
					Utilities::normalize(ray2Vector);

					cv::Point3f interPoint;

					bool ok = Utilities::line_lineIntersection(camera1.position, ray1Vector, camera2.position, ray2Vector, interPoint);


					if (!ok)
						continue;

					//get pixel color for the second camera view
					color2 = Utilities::matGet3D(colorImgs[cam2index], cam2Pixs[c2].x, cam2Pixs[c2].y);
					pointcloud_tresh.push_back(interPoint);
					color_tresh.push_back((color1 + color2) / 2);
				}

			}

		}
	}
	system("cls");
	std::cout << "Computing 3D Cloud  100%\n";

}

void UnderworldRcns::decodeTwoGroupOfImg(const vector<string>& imagelist, VirtualCamera& camera1, VirtualCamera& camera2, int i, int j, vector<Point3f>& pointcloud_tresh, vector<Vec3f>& color_tresh) {
	int numberOfPatternImages = 40;
	vector<Mat> color;
	color.resize(2);
	color[0] = imread(imagelist[numberOfPatternImages], IMREAD_COLOR);
	color[1] = imread(imagelist[2 * numberOfPatternImages + 2], IMREAD_COLOR);

	camera1.position = cv::Point3f(0, 0, 0);
	cam2WorldSpace(camera1, camera1.position);
	camera2.position = cv::Point3f(0, 0, 0);
	cam2WorldSpace(camera2, camera2.position);
	vector<vector<cv::Point>> camPixels1;
	vector<vector<cv::Point>> camPixels2;
	Tools::loadCamsPixelsForReconstuction(camPixels1, i);
	Tools::loadCamsPixelsForReconstuction(camPixels2, j);
	triangulation(color, camPixels1, camera1, camPixels2, camera2, 0, 1, pointcloud_tresh, color_tresh);
}
