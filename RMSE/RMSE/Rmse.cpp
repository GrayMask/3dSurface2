#include <mrpt/math/ransac_applications.h>
#include "Rmse.h"
using namespace mrpt::math;
using namespace std;

double Rmse::calcRmse(TPlane& plane, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs) {
	int sz = xs.size();
	double sum = 0;
	for (int i = 0; i < sz; i++) {
		TPoint3D point(xs[i], ys[i], zs[i]);
		double d = plane.distance(point);
		sum += (d*d);
	}
	double rmse = sqrt(sum/sz);
	return rmse;
}