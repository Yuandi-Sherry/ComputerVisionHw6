#include <stdio.h>
#include <tchar.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "CImg.h"
#include "Matrix.h"
#include "Pano.h"

using namespace std;
using namespace cimg_library;

extern "C" {
#include <vl/generic.h>
#include <vl/stringop.h>
#include <vl/pgm.h>
#include <vl/sift.h>
#include <vl/getopt_long.h>
#include <vl/kdtree.h>
#include <vl/random.h>
#include <vl/mathop.h>
};


#ifndef _RANSAC_H_
#define _RANSAC_H_
class Ransac
{
public:
	Ransac(const vector<pair<int, int> > & matchedPairsVec, const vector<vector <VlSiftKeypoint> > & keyPoints,
		const vector<vector < vector <float> > > & descriptors, int, int, vector< CImg<unsigned char> > & imgs);
	~Ransac();
	Matrix* getBestH();
private:
	double confidence = 0.99;
	double inlierRatio = 0.2;
	double epsilon = 1.5;
	Matrix* BestH;

};


#endif // !_RANSAC_H_
