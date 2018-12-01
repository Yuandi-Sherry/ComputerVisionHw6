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
	vector<vector <VlSiftKeypoint> > keyPoints; // ÿ��ͼƬ������������
	vector<vector < vector <float> > > descriptors; // ÿ��ͼƬ������������
	vector<pair<int, int> > matchedPairsVec; // ���ƥ�������
	void getMatchedPairs(int imgId);
	void getFeatures();
	void updateFeatures(int imgId);
	void warping(Matrix HMat, int img1Id, int img2Id);
	void doPairsMatching(int img1Id, int img2Id);
	CImg<unsigned char> resultImage;
	int left[17]; // ά��ÿ��ͼƬ�任���x�ڽ��ͼ�е���Сֵ
	int right[17];
};