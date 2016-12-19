#include <opencv2/core.hpp>;
using namespace cv;
#ifndef TOOLS_H
#define TOOLS_H
class Tools
{
public:
	static int copyFile(const String fromName, const String toName);
};
#endif