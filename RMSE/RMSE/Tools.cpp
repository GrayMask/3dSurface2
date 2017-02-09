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