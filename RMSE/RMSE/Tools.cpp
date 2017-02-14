#include <mrpt/math/ransac_applications.h>
#include "Tools.h"
#include <mrpt/random.h>

using namespace mrpt::math;
using namespace std;
using namespace mrpt::random;

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

void Tools::readPointCloud(char* filename, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs) {
	ifstream in(filename);
	if (!in.is_open())
	{
		cout << "Error opening file";
		return;
	}
	if (goWithLine(in, 3)) {
		string tmp;
		int count;
		in >> tmp >> tmp >> count;
		xs.resize(count);
		ys.resize(count);
		zs.resize(count);
		if (goWithLine(in, 8)) {
			for (int i = 0; i < count; i++) {
				in >> xs[i];
				in >> ys[i];
				in >> zs[i];
				goWithLine(in, 1);
			}
		}
	}
	return;
}

void Tools::readPointCloud(CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs) {
	randomGenerator.randomize();

	// Generate random points:
	// ------------------------------------

	const size_t N_plane = 300;
	const size_t N_noise = 300;

	const double PLANE_EQ[4] ={ 0,-1,1, +2 };

	for (size_t i = 0; i<N_plane; i++)
	{
		const double xx = randomGenerator.drawUniform(-3, 3) + 5 * cos(0.4);
		const double yy = randomGenerator.drawUniform(-3, 3) + 5 * sin(0.4);
		const double zz = -(PLANE_EQ[3] + PLANE_EQ[0] * xx + PLANE_EQ[1] * yy) / PLANE_EQ[2];
		xs.push_back(xx);
		ys.push_back(yy);
		zs.push_back(zz);
	}

	for (size_t i = 0; i<N_noise; i++)
	{
		xs.push_back(randomGenerator.drawUniform(-7, 7));
		ys.push_back(randomGenerator.drawUniform(-7, 7));
		zs.push_back(randomGenerator.drawUniform(-7, 7));
	}
}