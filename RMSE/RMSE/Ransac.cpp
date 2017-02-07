#include <mrpt/math/ransac_applications.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
#include "Ransac.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

void Ransac::TestRANSACPlanes(vector<pair<size_t, TPlane>>& detectedPlanes, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs)
{
	// Run RANSAC
	// ------------------------------------
	const double DIST_THRESHOLD = 0.05;

	CTicTac	tictac;

	ransac_detect_3D_planes(xs, ys, zs, detectedPlanes, DIST_THRESHOLD, 40);

	// Display output:
	cout << "RANSAC method: ransac_detect_3D_planes" << endl;
	cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
	cout << " " << detectedPlanes.size() << " planes detected." << endl;
}