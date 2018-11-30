#include <stdio.h>
#include <tchar.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "CImg.h"
#include "Matrix.h"

using namespace cv;
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

int main(int argc, _TCHAR* argv[])
{
	// ��ȡ��ߺ��ұ�����ͼƬ
	char ImagePath1[6] = "1.bmp"; 
	char ImagePath2[6] = "2.bmp";
	CImg<unsigned char> Image1(ImagePath1);
	CImg<unsigned char> Image2(ImagePath2);
	CImg<unsigned char> resultImage(Image1.width() + Image2.width(), Image1.height(), 1, 3);

	CImg<unsigned char> spherical1(Image1.width(), Image1.height(), 1, 3);

	cimg_forXY(Image1, x, y) {
		int xCenter = Image1.width() / 2;
		int yCenter = Image1.height() / 2;
		int X = x - xCenter;
		int Y = y - yCenter;
		double R = sqrt(pow(X, 2)/* + pow(Y, 2)*/ + pow(800, 2)); // focus = 800?
		double xHat = X / R;
		double yHat = Y / R;
		double zHat = 800 / R;
		double r2 = pow(xHat, 2) + pow(yHat, 2);
		double k1 = 0.05;
		// int xdPrime = (xHat/abs(xHat))*(abs(xHat) / zHat - r2 * k1) * 800 + xCenter;
		int ydPrime = (yHat / abs(yHat))*(abs(yHat) / zHat - r2 * k1) * 800 + yCenter;
		if (/*xdPrime < spherical1.width() && xdPrime >= 0 && */ydPrime < spherical1.height() && ydPrime >= 0) {
			//cout << "x " << x << " y " << y << "; xPrime " << xdPrime << " yPrime " << ydPrime << "r2" << r2 << endl;
			spherical1(x, ydPrime, 0, 0) = Image1(x, y, 0, 0);
			spherical1(x, ydPrime, 0, 1) = Image1(x, y, 0, 1);
			spherical1(x, ydPrime, 0, 2) = Image1(x, y, 0, 2);
		}
	}
	Image1 = spherical1;
	spherical1.save("test1.png");
	CImg<unsigned char> spherical2(Image2.width(), Image2.height(), 1, 3);
	cimg_forXY(spherical2, x, y) {
		spherical2(x, y, 0, 0) = 0;
		spherical2(x, y, 0, 1) = 0;
		spherical2(x, y, 0, 2) = 0;
	}

	cimg_forXY(Image2, x, y) {
		int xCenter = Image2.width() / 2;
		int yCenter = Image2.height() / 2;
		int X = x - xCenter;
		int Y = y - yCenter;
		double R = sqrt(pow(X, 2)/* + pow(Y, 2)*/ + pow(800, 2)); // focus = 800?
		double xHat = X / R;
		double yHat = Y / R;
		double zHat = 800 / R;
		double r2 = pow(xHat, 2) + pow(yHat, 2);
		double k1 = 0.1;
		// int xdPrime = (xHat / zHat + r2 * k1) * 800 + xCenter;
		int ydPrime = (yHat / abs(yHat))*(abs(yHat) / zHat - r2 * k1) * 800 + yCenter;
		//  xOrigin = ((x - xCenter)/800 - r2*k1)*(800/R)*R + xCenter;
		if (/*xdPrime < spherical2.width() && xdPrime >= 0 &&*/ ydPrime < spherical2.height() && ydPrime >= 0) {
			//cout << "x " << x << " y " << y << "; xPrime " << xdPrime << " yPrime " << ydPrime << "r2" << r2 << endl;
			spherical2(x, ydPrime, 0, 0) = Image2(x, y, 0, 0);
			spherical2(x, ydPrime, 0, 1) = Image2(x, y, 0, 1);
			spherical2(x, ydPrime, 0, 2) = Image2(x, y, 0, 2);
		}
	}
	Image2 = spherical2;
	Image2.save("sph2.png");
	cimg_forXY(Image1, x, y) {
		resultImage(x, y, 0, 0) = Image1(x, y, 0,0);
		resultImage(x, y, 0, 1) = Image1(x, y, 0, 1);
		resultImage(x, y, 0, 2) = Image1(x, y, 0, 2);
		resultImage(x+Image1.width(), y, 0,0) = Image2(x, y, 0,0);
		resultImage(x + Image1.width(), y, 0,1) = Image2(x, y, 0,1);
		resultImage(x + Image1.width(), y, 0,2) = Image2(x, y, 0,2);
	}
	// ȷ��octave��Ŀ��ÿ��octave��level��Ŀ
	//int noctaves = log(sqrt(Image->height*Image->width)) / log(2);
	int noctaves = log(sqrt(Image1.height()*Image1.width())) / log(2) - 3;
	//int noctaves = 4;
	cout << "noctaves " << noctaves << endl;
	int nlevels = 2, o_min = 0; // octave��Ŀ��ÿ��octave�㼶��Ŀ
	// �½�����ͼƬ������һά����
	vl_sift_pix *ImageData1 = new vl_sift_pix[Image1.height()*Image1.width()]; 
	vl_sift_pix *ImageData2 = new vl_sift_pix[Image2.height()*Image2.width()];
	// ��ͼƬ��Ϣд�����ؾ���
	unsigned char *Pixel; 
	for (int i = 0; i < Image1.height(); i++)
	{
		for (int j = 0; j < Image1.width(); j++)
		{
			int gray = Image1(j, i, 0, 0) * 0.299 + Image1(j, i, 0, 1) * 0.587 + Image1(j, i, 0, 2) * 0.114;
			ImageData1[i*Image1.width() + j] = gray;
		}
	}
	for (int i = 0; i < Image2.height(); i++)
	{
		for (int j = 0; j < Image2.width(); j++)
		{
			int gray = Image2(j, i, 0, 0) * 0.299 + Image2(j, i, 0, 1) * 0.587 + Image2(j, i, 0, 2) * 0.114;
			ImageData2[i*Image2.width() + j] = gray;
		}
	}


	VlSiftFilt *SiftFilt = NULL;
	/**
	 * width	image width.
	 * height	image height.
	 * noctaves	number of octaves.
	 * nlevels	number of levels per octave.
	 * o_min	first octave index.*/
	SiftFilt = vl_sift_new(Image1.width(), Image1.height(), noctaves, nlevels, o_min);
	// ������ͼ�ֱ�ͳ����������������	
	// ��¼SIFT��õ�������
	float *data1 = NULL;
	float *data2 = NULL;
	/* The function starts processing a new image by computing its Gaussian scale space at the lower octave. It also empties the internal keypoint buffer.
	   Returns error code. The function returns VL_ERR_EOF if there are no more octaves to process.*/
	// ����������
	vector<VlSiftKeypoint> keyPoints1;
	vector<VlSiftKeypoint> keyPoints2;
	// ����������
	vector< vector <float> > descriptorVec1; 
	vector< vector <float> > descriptorVec2;
	// ��������Ŀ
	int countOfDes1 = 0;
	int countOfDes2 = 0;
	// ����ͼ����SIFT��������ȡ
	if (vl_sift_process_first_octave(SiftFilt, ImageData1) != VL_ERR_EOF)
	{
		while (1)
		{
			//����ÿ���еĹؼ���
			/** 
			 * The function detect keypoints in the current octave filling the internal keypoint buffer. Keypoints can be retrieved by vl_sift_get_keypoints().
			 * Parameters f	SIFT filter.
			 */
			vl_sift_detect(SiftFilt);
			//����������ÿ����
			VlSiftKeypoint *pKeyPoint = SiftFilt->keys; // ����������
			 // ��������������
			for (int i = 0; i < SiftFilt->nkeys; i++) // ����ÿһ��������
			{
				VlSiftKeypoint TemptKeyPoint = *pKeyPoint; // ��ǰ������
				pKeyPoint++;
				//���㲢����ÿ����ķ���
				double angles[4]; 
				/**
				 * The function computes the orientation(s) of the keypoint k.
				 * The function returns the number of orientations found (up to four). 
				 * The orientations themselves are written to the vector angles.
				 */
				int angleCount = vl_sift_calc_keypoint_orientations(SiftFilt, angles, &TemptKeyPoint); // ����ĸ�
				for (int j = 0; j < angleCount; j++)  // �������еķ���
				{
					keyPoints1.push_back(TemptKeyPoint);
					double TemptAngle = angles[j];
					//����ÿ�����������
					float *Descriptors = new float[128]; 
					/**
					 * The function computes the SIFT descriptor of the keypoint k of orientation TemptAngle. 
					 * The function fills the buffer Descriptors which must be large enough to hold the descriptor.
					 */
					vl_sift_calc_keypoint_descriptor(SiftFilt, Descriptors, &TemptKeyPoint, TemptAngle);// ��õ�ǰ�����������
					// ��������������
					vector<float> copyOfDes;
					for (int p = 0; p < 128; p++) {
						copyOfDes.push_back(*(Descriptors + p));
					}
					descriptorVec1.push_back(copyOfDes);
					countOfDes1++;
					delete[] Descriptors;
					Descriptors = NULL;
				}
			}
			//��һ��
			/**
			 * The function computes the next octave of the Gaussian scale space. 
			 * Notice that this clears the record of any feature detected in the previous octave.
			 */
			if (vl_sift_process_next_octave(SiftFilt) == VL_ERR_EOF)
			{
				data1 = new float[128 * countOfDes1]; // ����������������ɵľ���
				for (unsigned int i = 0; i < countOfDes1; i++)
					std::copy(descriptorVec1[i].begin(), descriptorVec1[i].end(), data1 + 128 * i);
				break;
			}
		}

	}

	// ����ͼ��ȡ������
	SiftFilt = vl_sift_new(Image2.width(), Image2.height(), noctaves, nlevels, o_min);
	if (vl_sift_process_first_octave(SiftFilt, ImageData2) != VL_ERR_EOF)
	{
		while (1) {
			vl_sift_detect(SiftFilt);
			VlSiftKeypoint *pKeyPoint = SiftFilt->keys; 
			for (int i = 0; i < SiftFilt->nkeys; i++) {
				VlSiftKeypoint TemptKeyPoint = *pKeyPoint;
				pKeyPoint++;
				double angles[4];
				int angleCount = vl_sift_calc_keypoint_orientations(SiftFilt, angles, &TemptKeyPoint); 
				for (int j = 0; j < angleCount; j++) {
					keyPoints2.push_back(TemptKeyPoint);
					double TemptAngle = angles[j];
					float *Descriptors = new float[128];
					vl_sift_calc_keypoint_descriptor(SiftFilt, Descriptors, &TemptKeyPoint, TemptAngle);
					vector<float> copyOfDes;
					for (int p = 0; p < 128; p++) {
						copyOfDes.push_back(*(Descriptors + p));
					}
					descriptorVec2.push_back(copyOfDes);
					countOfDes2++;
					delete[] Descriptors;
					Descriptors = NULL;
				}
			}
			if (vl_sift_process_next_octave(SiftFilt) == VL_ERR_EOF)
			{
				data2 = new float[128 * countOfDes2]; 
				for (unsigned int i = 0; i < countOfDes2; i++)
					std::copy(descriptorVec2[i].begin(), descriptorVec2[i].end(), data2 + 128 * i);
				break;
			}
		}
	}

	// ����kdTree��ʹ����ͼ������
	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);
	vl_kdforest_build(forest, countOfDes1, data1);
	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	VlKDForestNeighbor neighbours[2];

	vector<pair<int, int>> matchedPairsVec;
	matchedPairsVec.clear();

	// compare
	int nvisited = 1;
	cout << "countOfDes1 " << countOfDes1 << "  countOfDes2 " << countOfDes2 << endl;
	for (int i = 0; i < countOfDes2; i++) {
		nvisited = vl_kdforestsearcher_query(searcher, neighbours, 2, data2 + 128*i);
		
		if (neighbours[0].distance < 0.8*neighbours[1].distance) {
			// todo ��ƥ������Դ�������
			matchedPairsVec.push_back(make_pair(neighbours[0].index, i));
			// result.draw_circle(keyPoints1[neighbours[0].index].x, keyPoints1[neighbours[0].index].y, 1, color);
			// result.draw_circle(keyPoints2[i].x + Image1.width(), keyPoints2[i].y, 1, color);
			// result.draw_line(keyPoints1[neighbours[0].index].x, keyPoints1[neighbours[0].index].y, keyPoints2[i].x + Image1.width(), keyPoints2[i].y, color);
		}
		
	}
	// �ͷſռ�
	vl_sift_delete(SiftFilt);

	// RANSAC
	double confidence = 0.99;
	double inlierRatio = 0.3;
	double epsilon = 1.5;
	int times = ceil(log(1-confidence) / log(1 - pow(inlierRatio, 4)));
	cout << "times: " << times << endl;
	Matrix bestH(3);
	int bestVote = 0;

	for (int t = 0; t < times; t++) {
		int fourPairs[4];
		for (int i = 0; i < 4; i++) {
			fourPairs[i] = rand() % matchedPairsVec.size();
		}
		// ����H
		double data[64];
		for (int i = 0; i < 64; i+=16) {
			data[i] = data[i+11] = keyPoints2[matchedPairsVec[fourPairs[i/16]].second].x;
			data[i+1] = data[i + 12] = keyPoints2[matchedPairsVec[fourPairs[i/16]].second].y;
			data[i + 2] = data[i + 13] = 1;
			data[i + 3] = data[i + 4] = data[i + 5] = data[i + 8] = data[i + 9] = data[i + 10] = 0;
			data[i+6] = -keyPoints1[matchedPairsVec[fourPairs[i / 16]].first].x * keyPoints2[matchedPairsVec[fourPairs[i / 16]].second].x;
			data[i + 7] = -keyPoints1[matchedPairsVec[fourPairs[i / 16]].first].x * keyPoints2[matchedPairsVec[fourPairs[i / 16]].second].y;
			data[i + 14] = -keyPoints1[matchedPairsVec[fourPairs[i / 16]].first].y * keyPoints2[matchedPairsVec[fourPairs[i / 16]].second].x;
			data[i + 15] = -keyPoints1[matchedPairsVec[fourPairs[i / 16]].first].y * keyPoints2[matchedPairsVec[fourPairs[i / 16]].second].y;
		}
		Matrix A(data, 8, 8);
		double vecB[8];
		for (int i = 0; i < 8; i += 2) {
			vecB[i] = keyPoints1[matchedPairsVec[fourPairs[i / 2]].first].x;
			vecB[i+1] = keyPoints1[matchedPairsVec[fourPairs[i / 2]].first].y;
		}

		Matrix b(vecB, 8, 1);
		Matrix result = A.Inverse()*b;
		
		double Hdata[9];
		for (int i = 0; i < 8; i++) {
			Hdata[i] = result.item[i];
		}
		Hdata[8] = 1;
		Matrix H(Hdata, 3, 3);

		// �������еĵ㣬��ͼ��ÿ������Ե�Ӧ����õ��Ľ����ƥ��Ľ����SSD���������������С��e������������ͶƱ
		int votes = 0;
		for (int i = 0; i < matchedPairsVec.size(); i++) {
			double xyData[3] = { keyPoints2[matchedPairsVec[i].second].x, keyPoints2[matchedPairsVec[i].second].y, 1};
			double xyDataPrime[3] = { keyPoints1[matchedPairsVec[i].first].x, keyPoints1[matchedPairsVec[i].first].y, 1};
			Matrix xy(xyData ,3, 1);
			Matrix xyPrime(xyDataPrime, 3, 1);
			Matrix result = H * xy;
			// ����SSD
			double mod = 0;
			for (int i = 0; i < 3; i++) {
				mod += pow(xyPrime.item[i] - result.item[i], 2);
			}
			if (mod < epsilon) {
				votes++;
			}
		}
		if (votes > bestVote) {
			cout << "votes > bestVote" << endl;
			bestVote = votes;
			bestH = H;
		}
	}

	cout << bestH << endl;

	// ���RANSAC֮���ɸѡ���
	for (int i = 0; i < matchedPairsVec.size(); i++) {
		double xyData[3] = { keyPoints2[matchedPairsVec[i].second].x, keyPoints2[matchedPairsVec[i].second].y, 1 };
		double xyDataPrime[3] = { keyPoints1[matchedPairsVec[i].first].x, keyPoints1[matchedPairsVec[i].first].y, 1 };
		Matrix xy(xyData, 3, 1);
		Matrix xyPrime(xyDataPrime, 3, 1);
		Matrix result = bestH * xy;
		double mod = 0;
		for (int i = 0; i < 3; i++) {
			mod += pow(xyPrime.item[i] - result.item[i], 2);
		}
		if (mod < epsilon) { // inliers
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			int color[3] = { r, g, b };
			resultImage.draw_line(keyPoints1[matchedPairsVec[i].first].x, keyPoints1[matchedPairsVec[i].first].y, keyPoints2[matchedPairsVec[i].second].x + Image1.width(), keyPoints2[matchedPairsVec[i].second].y, color);
		}
	}
	// spherical coordinate ӳ�䵽������
	// ʹ��Image1 test
	
	// warping
	cimg_forXY(Image2, x, y) {
		if (Image2(x, y, 0, 0) != 0 || Image2(x, y, 0, 1) != 0|| Image2(x, y, 0, 2) != 0) {
			// ����任����±�
		// ��ǰӳ��
			double data[3] = { x, y, 1 };
			Matrix xy(data, 3, 1);
			Matrix xyPrime = bestH * xy;
			//		if (xyPrime.item[2] != 1) {
				//		cout << "zz" << endl;
					//}
			int xPrime = round(xyPrime.item[0]);
			int yPrime = round(xyPrime.item[1]);
			if (xPrime < resultImage.width() && xPrime >= 0 && yPrime >= 0 && yPrime < resultImage.height()) {
				resultImage(xPrime, yPrime, 0, 0) = Image2(x, y, 0, 0);
				resultImage(xPrime, yPrime, 0, 1) = Image2(x, y, 0, 1);
				resultImage(xPrime, yPrime, 0, 2) = Image2(x, y, 0, 2);
			}
		}
	}
	
	// test.save("test.png");

	// result.draw_circle(keyPoints1[neighbours[0].index].x, keyPoints1[neighbours[0].index].y, 1, color);
	// result.draw_circle(keyPoints2[i].x + Image1.width(), keyPoints2[i].y, 1, color);
	// 
	// ����������� m = ceil(log(1 - confidence) / log(1 - inliner_Ratio^Npairs));
	// ����vector�洢ÿ�ε����õ���H���󣬲�����int����ά��ÿ��������ڵ�����Ʊ����
	// ���ѡ��matchedPairsVec�е��ĸ��±�
	// ����H����
	// �������еĵ㣬��ͼ��ÿ������Ե�Ӧ����õ��Ľ����ƥ��Ľ����SSD���������������С��e������������ͶƱ
	// ����Ʊ�����飬ѡ����Ʊ���ģ���ΪH
	// ��HӦ������ͼ������任��ÿ�����λ��
	// ��ʽ����ͼ��x'= H*��ͼ��x
	


	// ��ʾ���
	//Image1.save("left1.png");
	//Image2.save("right1.png");
	resultImage.save("stitching.png");
	// cvDestroyAllWindows();
	return 0;
}