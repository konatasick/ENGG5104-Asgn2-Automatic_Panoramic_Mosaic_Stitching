///////////////////////////////////////////////////////////////////////////
//
// NAME
//  FeatureAlign.h -- image registration using feature matching
//
// SEE ALSO
//  FeatureAlign.h      longer description
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "FeatureAlign.h"
#include "P3Math.h"
#include <math.h>
#include <time.h>



/******************* TO DO *********************
 * alignImagePair:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *		f: focal length
 *		width: image width
 *		height: image height
 *		nRANSAC: number of RANSAC iterations
 *		RANSACthresh: RANSAC distance threshold
 *		M: transformation matrix (output)
 *	OUTPUT:
 *		repeat for nRANSAC iterations:
 *			choose a minimal set of feature matches
 *			estimate the transformation implied by these matches
 *			count the number of inliers
 *		for the transformation with the maximum number of inliers,
 *		compute the least squares motion estimate using the inliers,
 *		and store it in M
 */
int findFeaturePairs(const FeatureSet &f1, const FeatureSet &f2, const vector<FeatureMatch> &matches, vector<int> &inliers)
{

	inliers.clear();
	int i1,i2;
	while (true)
	{
		i1 = rand() % matches.size();
		if (matches[i1].id != -1)
			break;
		
	}
	while (true)
	{
		i2 = rand() % matches.size();
		if (i2 != i1 & matches[i2].id != -1)
			break;		
	}

	//set this two pairs as inliers
	inliers.push_back(f1[i1].id-1);
	inliers.push_back(f1[i2].id-1);
	//printf("%d %d random pairs, ", f1[i1].id, f1[i2].id);
	return 0;
}
int alignImagePair(const FeatureSet &f1, const FeatureSet &f2,
			  const vector<FeatureMatch> &matches, MotionModel m,
			  float f, int width, int height,
			  int nRANSAC, double RANSACthresh, CTransform3x3& M, vector<int> &inliers)
{
	// BEGIN TODO
	// write this entire method
	int countMatch = 0;
		for (int i=0; i<matches.size(); i++)
			if (matches[i].id != -1)
				countMatch++;
		if (countMatch < 2)
	return 0;

	srand((unsigned)time(NULL));
	int count,maxCount = 0;

	for(int i = 0; i < nRANSAC; i++)
	{
		CTransform3x3 tempM;
		vector<int> tempInliers;
		
		//randomly select two feature pairs

		findFeaturePairs(f1, f2, matches, tempInliers);

		leastSquaresFit(f1, f2, matches, m, f, width, height, tempInliers, tempM);
		//printf("%d loop\n", i);
		//printf("\n%f %f %f\n", tempM[0][0], tempM[0][1], tempM[0][2]);
		//printf("%f %f %f\n", tempM[1][0], tempM[1][1], tempM[1][2]);
		//printf("%f %f %f\n", tempM[2][0], tempM[2][1], tempM[2][2]);
		count = countInliers(f1, f2, matches, m, f, width, height, tempM, RANSACthresh, tempInliers);

		//printf("%d inliers\n", count);
		if (count > maxCount)
		{
			maxCount = count;
			inliers = tempInliers;
			//printf("%d inliers\n", maxCount);
		}
	}
	//keep largest set of inliers
	leastSquaresFit(f1, f2, matches, m, f, width, height, inliers, M);
	//printf("%d inliers\n", inliers.size());
	//printf("\n%f %f %f\n", M[0][0], M[0][1], M[0][2]);
	//printf("%f %f %f\n", M[1][0], M[1][1], M[1][2]);
	//printf("%f %f %f\n", M[2][0], M[2][1], M[2][2]);
	// END TODO

	return 0;
}

/******************* TO DO *********************
 * countInliers:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *		f: focal length
 *		width: image width
 *		height: image height
 *		M: transformation matrix
 *		RANSACthresh: RANSAC distance threshold
 *		inliers: inlier feature IDs
 *	OUTPUT:
 *		transform the features in f1 by M
 *
 *		count the number of features in f1 for which the transformed
 *		feature is within Euclidean distance RANSACthresh of its match
 *		in f2
 *
 *		store these features IDs in inliers
 *
 *		this method should be similar to evaluateMatch from project 1,
 *		except you are comparing each distance to a threshold instead
 *		of averaging them
 */
CVector3 position(const Feature key, float f, int width, int height)
{
	CVector3 p;
	p[0] = key.x-0.5f*width;
	p[1] = key.y-0.5f*height;
	p[2] = f;
	return p;
}

int countInliers(const FeatureSet &f1, const FeatureSet &f2,
				 const vector<FeatureMatch> &matches, MotionModel m,
				 float f, int width, int height,
				 CTransform3x3 M, double RANSACthresh, vector<int> &inliers)
{
	inliers.clear();
	int count = 0;

	for (unsigned int i=0; i<f1.size(); i++) {
		// BEGIN TODO
		// determine if the ith feature in f1, when transformed by M,
		// is within RANSACthresh of its match in f2 (if one exists)
		//
		// if so, increment count and append i to inliers
		double dist = 0;
		CVector3 p1, p2, Rp1;

		if (matches[i].id < 0)
			continue;

		p1 = position(f1[i], f, width, height);
		p2 = position(f2[matches[i].id-1], f, width, height);
		Rp1 = M * p1;
		dist  = sqrt(pow(p2[0]-Rp1[0],2) + pow(p2[1]-Rp1[1],2));
		
		if(dist < RANSACthresh)
		{
			//printf("%f.\n",dist);
			inliers.push_back(f1[i].id-1);
			count++; 
		}
		// END TODO
	}

	return count;
}

/******************* TO DO *********************
 * leastSquaresFit:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *		f: focal length
 *		width: image width
 *		height: image height
 *		inliers: inlier feature IDs
 *		M: transformation matrix (output)
 *	OUTPUT:
 *		compute the transformation from f1 to f2 using only the inliers
 *		and return it in M
 */
int leastSquaresFit(const FeatureSet &f1, const FeatureSet &f2,
					const vector<FeatureMatch> &matches, MotionModel m,
					float f, int width, int height,
					const vector<int> &inliers, CTransform3x3& M)
{
	// BEGIN TODO
	// write this entire method
	CTransform3x3 U, S, V, A;
	CVector3 p, pp;
    for (int i = 0; i < 3; i++)
		A[i][i] = 0;
	//calculate A
	for (int i = 0; i<inliers.size(); i++)
	{
		int pPos = inliers[i];
		int ppPos = matches[pPos].id - 1;
		//printf("%d and %d.\n",pPos,ppPos);
		p = position(f1[pPos], f, width, height);
		
		pp = position(f2[ppPos], f, width, height);

		A = A + p * pp;
	}

	svd(A, U, S, V);
	M = V * U.Transpose();

	// END TODO

	return 0;
}
