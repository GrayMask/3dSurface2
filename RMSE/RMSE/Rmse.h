#include <mrpt/math/ransac_applications.h>
using namespace mrpt::math;
#ifndef RMSE_H
#define RMSE_H
class Rmse
{
public:
	double calcRmse(TPlane& plane, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs);
};
#endif