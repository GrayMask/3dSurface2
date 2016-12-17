#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <io.h>
#include <direct.h>
#include "GrayCodePattern.h"
#include "Const.h"
#include "Path.h"

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
	if (_access(expr_dir.c_str(), 6) == -1)
	{
		int result = _mkdir(expr_dir.c_str());
		cout << result << endl;
	}
	// Turning off autofocus
	cap1.set(CAP_PROP_SETTINGS, 1);
	int j = 0;
	while (true) {
		int key = waitKey(1);
		if (key == 113) {
			break;
		}
		else if (key == 13) {
			char* imagesGroupDirTemp = new char[images_group_dir_length];
			sprintf(imagesGroupDirTemp, images_group_dir, j);
			String imagesDir = expr_dir + String(imagesGroupDirTemp);
			if (_access(imagesDir.c_str(), 6) == -1)
			{
				cout << imagesDir << endl;
				int result = _mkdir(imagesDir.c_str());
				cout << result << endl;
			}
			FileStorage fs(imagesDir + imagesName_file, FileStorage::WRITE);
			fs << "imagelist" << "[";
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
						String imagesFile = imagesDir + images_file + name.str() + imgType;
						save1 = imwrite(imagesFile, frame1);
						if (save1)
						{
							cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
							fs << imagesFile;
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
			fs << "]";
			destroyWindow("cam1");
			j++;
		}
	}
	
	// the camera will be deinitialized automatically in VideoCapture destructor
}


