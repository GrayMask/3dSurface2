#include <opencv2/core.hpp>;
using namespace cv;
using namespace std;
#ifndef TOOLS_H
#define TOOLS_H
class Tools
{
public:
	static int copyFile(const String fromName, const String toName);

	static int writePic(const Mat& im, const String& fname);

	static int writeGroupNumFile(const int groupNum);

	static int readGroupNumFile(int& groupNum);

	static int readStringList(String& filename, vector<string>& l);

	static int getSFMResult(const int count, Mat& R, Mat& T);
};
#endif