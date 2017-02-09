#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <io.h>
#include <direct.h>
#include "GrayCodePattern.h"
#include "Path.h"
#include "Const.h"
#include "Tools.h"

using namespace cv;
using namespace std;

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
	VideoCapture cap1(0);
	if (!cap1.isOpened())
	{
		// check if cam1 opened
		cout << "cam1 not opened!" << endl;
		exit(-1);
	}
	if (_access((root_dir + expr_dir).c_str(), 6) == -1)
	{
		int result = _mkdir((root_dir + expr_dir).c_str());
	}
	// Turning off autofocus
	cap1.set(CAP_PROP_SETTINGS, 1);
	cap1.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);
	VideoCapture cap2(1);
	if (isStereoCamera) {
		cap2.set(CAP_PROP_SETTINGS, 1);
		cap2.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
		cap2.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);
	}
	int j = 0;
	while (true) {
		int key = waitKey(1);
		if (key == 113) {
			break;
		}
		else if (key == 13) {
			char* imagesGroupDirTemp = new char[images_group_dir_length];
			sprintf(imagesGroupDirTemp, images_group_dir, j);
			String imagesDir1 = root_dir + expr_dir + String(imagesGroupDirTemp);
			if (_access(imagesDir1.c_str(), 6) == -1)
			{
				int result = _mkdir(imagesDir1.c_str());
			}
			FileStorage fs1(imagesDir1 + imagesName_file, FileStorage::WRITE);
			fs1 << "imagelist" << "[";
			String imagesDir2;
			FileStorage fs2;
			if (isStereoCamera) {
				j++;
				sprintf(imagesGroupDirTemp, images_group_dir, j);
				imagesDir2 = root_dir + expr_dir + String(imagesGroupDirTemp);
				if (_access(imagesDir2.c_str(), 6) == -1)
				{
					int result = _mkdir(imagesDir2.c_str());
				}
				fs2.open(imagesDir2 + imagesName_file, FileStorage::WRITE);
				fs2 << "imagelist" << "[";
			}
			int i = 0;
			while (i < (int)pattern.size())
			{
				cout << "Waiting to save image number " << i + 1 << endl << "Press any key to acquire the photo" << endl;
				imshow("Pattern Window", pattern[i]);
				Mat frame1, frame2;
				while (1)
				{
					cap1 >> frame1;  // get a new frame from camera 1
					if (isStereoCamera) {
						cap2 >> frame2;
					}
					//imshow("cam1", frame1);
					int key = waitKey(1);
					if (key == 115)
					{
						bool save1, save2 = false;
						ostringstream name;
						name << i + 1;
						String imagesFile1 = imagesDir1 + images_file + name.str() + imgType;
						String imagesFile2;
						cout << imagesFile1 << endl;
						save1 = imwrite(imagesFile1, frame1);
						if (isStereoCamera) {
							imagesFile2 = imagesDir2 + images_file + name.str() + imgType;
							cout << imagesFile2 << endl;
							save2 = imwrite(imagesFile2, frame2);
						}
						if (!isStereoCamera) {
							if (save1) {
								cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
								fs1 << imagesFile1;
								i++;
								break;
							}
							cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path" << endl << endl;
						}
						else {
							if (save1 && save2) {
								cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
								fs1 << imagesFile1;
								cout << "pattern cam2 images number " << i + 1 << " saved" << endl << endl;
								fs2 << imagesFile2;
								i++;
								break;
							}
							cout << "pattern cam1 or cam2 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path" << endl << endl;
						}
					}
				}
			}
			fs1 << "]"; 
			if (isStereoCamera) {
				fs2 << "]";
			}
			//destroyWindow("cam1");
			j++;
		}
	}
	Tools::writeGroupNumFile(j);
}


