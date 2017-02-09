#include <mrpt/math/ransac_applications.h>
#include "Ransac.h"
#include "Rmse.h"
#include "Tools.h"
using namespace mrpt::math;
using namespace std;

void main() {
	Ransac ransac;
	Rmse rmse;
	vector<pair<size_t, TPlane>> planes;

	CVectorDouble xs, ys, zs;
	Tools::readPointCloud("E:\\japan\\research\\3dsurface2\\3dSurface2\\expr9\\01pointCloud.ply", xs, ys, zs);
	ransac.TestRANSACPlanes(planes, xs, ys, zs);
	double result = rmse.calcRmse(planes[0].second, xs, ys, zs);
	cout << result << endl;
}