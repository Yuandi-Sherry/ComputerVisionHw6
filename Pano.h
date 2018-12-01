#pragma once
#include "CImg.h"
#include "Matrix.h"
#include <string>
#include <vector>
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

using namespace std;
using namespace cimg_library;
class Pano
{
public:
	Pano(const string[], int n);
	~Pano();
	
private:
	vector< CImg<unsigned char> > imgs;
	int num;
	vector<vector <VlSiftKeypoint> > keyPoints; // 每幅图片的特征点序列
	vector<vector < vector <float> > > descriptors; // 每幅图片的特征点序列
	vector<pair<int, int> > matchedPairsVec; // 获得匹配的数对
	void getMatchedPairs(int imgId);
	void getFeatures();
	void updateFeatures(int imgId);
	void warping(Matrix HMat, int img1Id, int img2Id);
	void doPairsMatching(int img1Id, int img2Id);
	CImg<unsigned char> resultImage;
	int left[17]; // 维护每个图片变换后的x在结果图中的最小值
	int right[17];
};