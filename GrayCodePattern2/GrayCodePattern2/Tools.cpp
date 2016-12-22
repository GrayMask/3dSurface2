#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>;
#include "Tools.h"
#include "Const.h"
#include "Path.h"

using namespace cv;
using namespace std;

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