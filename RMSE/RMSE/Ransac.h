#include <mrpt/math/ransac_applications.h>
using namespace mrpt::math;
using namespace std;
#ifndef RANSAC_H
#define RANSAC_H
class Ransac
{
public:
	void TestRANSACPlanes(vector<pair<size_t, TPlane>>& detectedPlanes, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs);
};
#endif