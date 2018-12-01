#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "CImg.h"
#include "Matrix.h"
#include "Pano.h"
#include "Ransac.h"

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


Ransac::Ransac(const vector<pair<int, int> > & matchedPairsVec, const vector<vector <VlSiftKeypoint> > & keyPoints,
	const vector<vector < vector <float> > > & descriptors, int img1Id, int img2Id, vector< CImg<unsigned char> > & imgs) {
	/* img1 is the reference, img2 is to be change*/
	// 计算所需次数
	int times = ceil(log(1 - confidence) / log(1 - pow(inlierRatio, 4)));
	Matrix bestH(3);
	int bestVote = 0;
	for (int t = 0; t < times; t++) {
		int fourPairs[4];
		for (int i = 0; i < 4; i++) {
			fourPairs[i] = rand() % matchedPairsVec.size();
		}
		// 计算H
		double data[64];
		for (int i = 0; i < 64; i += 16) {
			data[i] = data[i + 11] = keyPoints[img2Id][matchedPairsVec[fourPairs[i / 16]].second].x;
			data[i + 1] = data[i + 12] = keyPoints[img2Id][matchedPairsVec[fourPairs[i / 16]].second].y;
			data[i + 2] = data[i + 13] = 1;
			data[i + 3] = data[i + 4] = data[i + 5] = data[i + 8] = data[i + 9] = data[i + 10] = 0;
			data[i + 6] = -keyPoints[img1Id][matchedPairsVec[fourPairs[i / 16]].first].x * keyPoints[img2Id][matchedPairsVec[fourPairs[i / 16]].second].x;
			data[i + 7] = -keyPoints[img1Id][matchedPairsVec[fourPairs[i / 16]].first].x * keyPoints[img2Id][matchedPairsVec[fourPairs[i / 16]].second].y;
			data[i + 14] = -keyPoints[img1Id][matchedPairsVec[fourPairs[i / 16]].first].y * keyPoints[img2Id][matchedPairsVec[fourPairs[i / 16]].second].x;
			data[i + 15] = -keyPoints[img1Id][matchedPairsVec[fourPairs[i / 16]].first].y * keyPoints[img2Id][matchedPairsVec[fourPairs[i / 16]].second].y;
		}
		Matrix A(data, 8, 8);
		double vecB[8];
		for (int i = 0; i < 8; i += 2) {
			vecB[i] = keyPoints[img1Id][matchedPairsVec[fourPairs[i / 2]].first].x;
			vecB[i + 1] = keyPoints[img1Id][matchedPairsVec[fourPairs[i / 2]].first].y;
		}

		Matrix b(vecB, 8, 1);
		Matrix result = A.Inverse()*b;

		double Hdata[9];
		for (int i = 0; i < 8; i++) {
			Hdata[i] = result.item[i];
		}
		Hdata[8] = 1;
		Matrix H(Hdata, 3, 3);

		// 遍历所有的点，左图的每个点乘以单应矩阵得到的结果和匹配的结果的SSD（作差求范数）如果小于e（参数），则投票
		int votes = 0;
		for (int i = 0; i < matchedPairsVec.size(); i++) {
			double xyData[3] = { keyPoints[img2Id][matchedPairsVec[i].second].x, keyPoints[img2Id][matchedPairsVec[i].second].y, 1 };
			double xyDataPrime[3] = { keyPoints[img1Id][matchedPairsVec[i].first].x, keyPoints[img1Id][matchedPairsVec[i].first].y, 1 };
			Matrix xy(xyData, 3, 1);
			Matrix xyPrime(xyDataPrime, 3, 1);
			Matrix result = H * xy;
			// 计算SSD
			double mod = 0;
			for (int i = 0; i < 3; i++) {
				mod += pow(xyPrime.item[i] - result.item[i], 2);
			}
			if (mod < epsilon) {
				votes++;
			}
		}
		if (votes > bestVote) {
			bestVote = votes;
			bestH = H;
		}
	}

	// testing
	/*CImg<unsigned char> result = CImg<unsigned char>(imgs[img1Id].width() + imgs[img2Id].width(), imgs[img1Id].height(), 1, 3);
	cimg_forXY(imgs[img2Id], x, y) {
		imgs[img1Id](x + imgs[img2Id].width() * (img2Id - 1), y, 0, 0) = imgs[img2Id](x, y, 0, 0);
		imgs[img1Id](x + imgs[img2Id].width() *  (img2Id - 1), y, 0, 1) = imgs[img2Id](x, y, 0, 1);
		imgs[img1Id](x + imgs[img2Id].width() *  (img2Id - 1), y, 0, 2) = imgs[img2Id](x, y, 0, 2);
	}*/
	// testing end

	/*for (int i = 0; i < matchedPairsVec.size(); i++) {
		double xyData[3] = { keyPoints[img2Id][matchedPairsVec[i].second].x, keyPoints[img2Id][matchedPairsVec[i].second].y, 1 };
		double xyDataPrime[3] = { keyPoints[img1Id][matchedPairsVec[i].first].x, keyPoints[img1Id][matchedPairsVec[i].first].y, 1 };
		Matrix xy(xyData, 3, 1);
		Matrix xyPrime(xyDataPrime, 3, 1);
		Matrix resultMat = bestH * xy;
		double mod = 0;
		for (int i = 0; i < 3; i++) {
			mod += pow(xyPrime.item[i] - resultMat.item[i], 2);
		}
		if (mod < epsilon) { // inliers
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			int color[3] = { r, g, b };
			//imgs[0].draw_line(keyPoints[img1Id][matchedPairsVec[i].first].x, keyPoints[img1Id][matchedPairsVec[i].first].y, keyPoints[img2Id][matchedPairsVec[i].second].x  + imgs[img2Id].width() * (img2Id-1), keyPoints[img2Id][matchedPairsVec[i].second].y, color);
		}
	}*/
	//imgs[0].display();
	BestH = new Matrix(bestH);
}

Ransac::~Ransac()
{
}

Matrix* Ransac::getBestH() {
	return BestH;
}