#include <mrpt/math/ransac_applications.h>
#include <mrpt/math/ransac.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
#include "Ransac.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

void  ransac3Dplane_fit(
	const CMatrixTemplateNumeric<double> &allData,
	const vector_size_t  &useIndices,
	vector< CMatrixTemplateNumeric<double> > &fitModels)
{
	ASSERT_(useIndices.size() == 3);

	TPoint3D  p1(allData(0, useIndices[0]), allData(1, useIndices[0]), allData(2, useIndices[0]));
	TPoint3D  p2(allData(0, useIndices[1]), allData(1, useIndices[1]), allData(2, useIndices[1]));
	TPoint3D  p3(allData(0, useIndices[2]), allData(1, useIndices[2]), allData(2, useIndices[2]));

	try
	{
		TPlane  plane(p1, p2, p3);
		fitModels.resize(1);
		CMatrixTemplateNumeric<double> &M = fitModels[0];

		M.setSize(1, 4);
		for (size_t i = 0; i<4; i++)
			M(0, i) = plane.coefs[i];
	}
	catch (exception &)
	{
		fitModels.clear();
		return;
	}
}

void ransac3Dplane_distance(
	const CMatrixTemplateNumeric<double> &allData,
	const vector< CMatrixTemplateNumeric<double> > & testModels,
	const double distanceThreshold,
	unsigned int & out_bestModelIndex,
	vector_size_t & out_inlierIndices)
{
	ASSERT_(testModels.size() == 1)
		out_bestModelIndex = 0;
	const CMatrixTemplateNumeric<double> &M = testModels[0];

	ASSERT_(size(M, 1) == 1 && size(M, 2) == 4)

		TPlane  plane;
	plane.coefs[0] = M(0, 0);
	plane.coefs[1] = M(0, 1);
	plane.coefs[2] = M(0, 2);
	plane.coefs[3] = M(0, 3);

	const size_t N = size(allData, 2);
	out_inlierIndices.clear();
	out_inlierIndices.reserve(100);
	for (size_t i = 0; i<N; i++)
	{
		const double d = plane.distance(TPoint3D(allData.get_unsafe(0, i), allData.get_unsafe(1, i), allData.get_unsafe(2, i)));
		if (d<distanceThreshold)
			out_inlierIndices.push_back(i);
	}
}

/** Return "true" if the selected points are a degenerate (invalid) case.
*/
bool ransac3Dplane_degenerate(
	const CMatrixTemplateNumeric<double> &allData,
	const mrpt::vector_size_t &useIndices)
{
	MRPT_UNUSED_PARAM(allData);
	MRPT_UNUSED_PARAM(useIndices);
	return false;
}

void Ransac::TestRANSACPlanes(vector<pair<size_t, TPlane>>& detectedPlanes, CVectorDouble& xs, CVectorDouble& ys, CVectorDouble& zs)
{
	// Run RANSAC
	// ------------------------------------
	const double DIST_THRESHOLD = 0.7;
	CTicTac	tictac;

		//math::RANSAC_Template<double>::execute(
		//	remainingPoints,
		//	ransac3Dplane_fit,
		//	ransac3Dplane_distance,
		//	ransac3Dplane_degenerate,
		//	DIST_THRESHOLD,
		//	3,  // Minimum set of points
		//	this_best_inliers,
		//	this_best_model,
		//	true, // Verbose
		//	0.999  // Prob. of good result
		//);
	ransac_detect_3D_planes(xs, ys, zs, detectedPlanes, DIST_THRESHOLD, 5000);
	// Display output:
	//cout << "RANSAC method: ransac_detect_3D_planes" << endl;
	cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
	cout << " " << detectedPlanes.size() << " planes detected." << endl;
}