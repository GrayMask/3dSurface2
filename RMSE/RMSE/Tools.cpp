#include <mrpt/math/ransac_applications.h>
#include "Tools.h"
#include <mrpt/random.h>

using namespace mrpt::math;
using namespace std;
using namespace mrpt::random;

void Tools::readPointCloud(char* filename, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs) {
	randomGenerator.randomize();

	const size_t N_plane = 300;
	const size_t N_noise = 300;

	const double PLANE_EQ[4] =
	{ 1,-3,1, -2 };
	xs.resize(N_plane + N_noise);
	ys.resize(N_plane + N_noise);
	zs.resize(N_plane + N_noise);
	int sz = 0;
	for (size_t i = 0; i<N_plane; i++)
	{
		const double xx = randomGenerator.drawUniform(-3, 3) + 5 * cos(0.4);
		const double yy = randomGenerator.drawUniform(-3, 3) + 5 * sin(0.4);
		const double zz = -(PLANE_EQ[3] + PLANE_EQ[0] * xx + PLANE_EQ[1] * yy) / PLANE_EQ[2];
		xs[sz] = xx;
		ys[sz] = yy;
		zs[sz] = zz;
		sz++;
	}

	for (size_t i = 0; i<N_noise; i++)
	{
		xs[sz] = randomGenerator.drawUniform(-7, 7);
		ys[sz] = randomGenerator.drawUniform(-7, 7);
		zs[sz] = randomGenerator.drawUniform(-7, 7);
		sz++;
	}
}