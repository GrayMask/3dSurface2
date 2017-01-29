#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>;
#include "Tools.h"
#include "Const.h"
#include "Path.h"

using namespace cv;
using namespace std;


#define cvQueryHistValue_1D( hist, idx0 ) \
    ((float)cvGetReal1D( (hist)->bins, (idx0)))

int Tools::copyFile(const String fromFile, const String toFile) {
	fstream in(fromFile, ios::in | ios::binary);
	if (!in)
	{
		cerr << "open " << fromFile << " failed" << endl;
		in.close();
		return 0;
	}

	fstream out(toFile, ios::out | ios::binary);
	if (!out)
	{
		cerr << "open " << toFile << " failed" << endl;
		out.close();
		return 0;
	}

	char temp;
	while (in.get(temp))
	{
		out << temp;
	}
	out << endl;
	cout << "success!" << endl;
	in.close();
	out.close();
}

int Tools::writePic(const Mat& im, const String& fname)
{
	ofstream ouF;
	ouF.open(fname.c_str());
	if (!ouF)
	{
		cerr << "failed to open the file : " << fname << endl;
		return 0;
	}
	for (int r = 0; r < im.rows; r++)
	{
		for (int c = 0; c < im.cols; c++) {
			ouF << im.at<float>(r, c);
			ouF << " ";
		}
		ouF << "\n";
	}
	ouF.close();
	return 1;
}

int Tools::writeGroupNumFile(const int groupNum)
{
	ofstream out(root_dir + groupNum_file);
	if (out.is_open())
	{
		out << groupNum;
		out.close();
	}
	else {
		return -1;
	}
	return 1;
}

int Tools::readGroupNumFile(int& groupNum)
{
	ifstream in(root_dir + groupNum_file);
	if (!in.is_open())
	{
		cout << "Error opening file";
		return -1;
	}
	in >> groupNum;
	return 1;
}

int Tools::readStringList(String& filename, vector<string>& l) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "failed to open " << filename << endl;
		return -1;
	}
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
	{
		cerr << "cam 1 images are not a sequence! FAIL" << endl;
		return -1;
	}
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
	{
		l.push_back((string)*it);
	}
	return 1;
}

bool goWithLine(ifstream & in, int line)
{
	int i;
	char buf[1024];
	for (i = 0; i < line; i++)
	{
		if (!in.getline(buf, sizeof(buf))) {
			return false;
		}
	}
	return true;
}

int Tools::getSFMResult(const int count, Mat& R, Mat& T) {
	R.create(3,3,CV_64F);
	T.create(3, 1, CV_64F);
	double num;
	ifstream in(root_dir + sfm_file);
	if (!in.is_open())
	{
		cout << "Error opening file";
		return 0;
	}
	ostringstream countStr;
	countStr << count;
	String targetPathStr = countStr.str() + imgType;
	const char* targetPath = targetPathStr.c_str();
	// goto line 19
	if (goWithLine(in, 19)) {
		do {
			char nowPath[1024];
			in.getline(nowPath, sizeof(nowPath));
			char *last = strrchr(nowPath, '\\') +1 ;
			if (strcmp(targetPath, last) == 0) {
				goWithLine(in, 2);
				for (int i = 0; i < 3; i++) {
					in >> num;
					T.at<double>(i, 0) = num;
				}
				goWithLine(in, 4);
				for (int i = 0; i < 9; i++) {
					in >> num;
					R.at<double>(i / 3, i % 3) = num;
				}
				return 1;
			}
		} while (goWithLine(in, 13));
	}
	return 0;
}

void Tools::myCalcHist(Mat gray_plane)
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