#include <mrpt/math/ransac_applications.h>
#include <string>
using namespace mrpt::math;
using namespace std;
#ifndef TOOLS_H
#define TOOLS_H
class Tools
{
public:
	static void readPointCloud(char* filename, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs);
};
#endif