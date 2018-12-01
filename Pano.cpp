#include "Pano.h"
#include "SphericalTrans.h"
#include "CImg.h"
#include "Ransac.h"
#include "Matrix.h"
#include <string>
#include <iostream>

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
Pano::Pano(const string filenames [], int n)
{
	num = n;
	imgs = new CImg<unsigned char>[n];
	for (int i = 0; i < n; i++) {
		CImg<unsigned char> tempImg(filenames[i].c_str());
		imgs[i] = tempImg;
	}
	// ����������任
	SphericalTrans st(imgs, num);
	getFeatures();
	getMatchedPairs(0,1);
	
	Ransac ransac(matchedPairsVec, keyPoints, descriptors, 0, 1, imgs);
	Matrix HMat( *(ransac.getBestH()));
	warping(HMat, 0, 1);
}

Pano::~Pano()
{
}

void Pano::getFeatures() {
	VlSiftFilt *SiftFilt = NULL;
	// �˲�����Ȼ����ͼƬ�ߴ�һ��
	int noctaves = log(sqrt(imgs[0].height()*imgs[0].width())) / log(2) - 3;
	int nlevels = 2, o_min = 0; // octave��Ŀ��ÿ��octave�㼶��Ŀ
	/* The function starts processing a new image by computing its Gaussian scale space at the lower octave. It also empties the internal keypoint buffer.
	   Returns error code. The function returns VL_ERR_EOF if there are no more octaves to process.*/

	// ����������
	//vector< vector <float> > descriptorVec1;
	vector< vector <float> > descriptorVec2;

	
	for (int k = 0; k < num; k++) {
		// �½�����ͼƬ������һά����
		SiftFilt = vl_sift_new(imgs[0].width(), imgs[0].height(), noctaves, nlevels, o_min);
		vl_sift_pix *ImageData = new vl_sift_pix[imgs[k].height()*imgs[k].width()];
		for (int i = 0; i < imgs[k].height(); i++) {
			for (int j = 0; j < imgs[k].width(); j++) {
				int gray = imgs[k](j, i, 0, 0) * 0.299 + imgs[k](j, i, 0, 1) * 0.587 + imgs[k](j, i, 0, 2) * 0.114;
				ImageData[i*imgs[k].width() + j] = gray;
			}
		}
		vector< VlSiftKeypoint > tempKeys; 
		vector< vector <float> > tempDes; // ��ǰͼƬ����������
		if (vl_sift_process_first_octave(SiftFilt, ImageData) != VL_ERR_EOF) {
			while (1){
				//����ÿ���еĹؼ���
				vl_sift_detect(SiftFilt);
				VlSiftKeypoint *pKeyPoint = SiftFilt->keys; // ����������
				 // ��������������
				for (int i = 0; i < SiftFilt->nkeys; i++) { // ����ÿһ��������
					VlSiftKeypoint TemptKeyPoint = *pKeyPoint; // ��ǰ������
					pKeyPoint++;
					//���㲢����ÿ����ķ���
					double angles[4];
					int angleCount = vl_sift_calc_keypoint_orientations(SiftFilt, angles, &TemptKeyPoint); // ����ĸ�
					for (int j = 0; j < angleCount; j++)  {// �������еķ���
						tempKeys.push_back(TemptKeyPoint);
						double TemptAngle = angles[j];
						//����ÿ�����������
						float *Descriptors = new float[128];
						vl_sift_calc_keypoint_descriptor(SiftFilt, Descriptors, &TemptKeyPoint, TemptAngle);// ��õ�ǰ�����������
						// ��������������
						vector<float> copyOfDes;
						for (int p = 0; p < 128; p++) {
							copyOfDes.push_back(*(Descriptors + p));
						}
						cout << "tempDes.push_back(copyOfDes);" << endl;
						tempDes.push_back(copyOfDes);
						delete[] Descriptors;
						Descriptors = NULL;
					}
				}
				//��һ��
				if (vl_sift_process_next_octave(SiftFilt) == VL_ERR_EOF)
				{
					// data1 = new float[128 * countOfDes1]; // ����������������ɵľ���
					// for (unsigned int i = 0; i < countOfDes1; i++)
						// std::copy(descriptorVec1[i].begin(), descriptorVec1[i].end(), data1 + 128 * i);
					break;
				}
			}

		}
		cout << "temp.size()" << tempDes.size() << endl;
		descriptors.push_back(tempDes);
		keyPoints.push_back(tempKeys);
	}
	cout << "descriptors.size() " <<  descriptors.size() << endl;
}

void Pano::getMatchedPairs(int img1Id, int img2Id) {

	float* data1;
	float* data2;
	// ����ͼ�е����ݴ洢����
	data1 = new float[128 * descriptors[img1Id].size()];
	data2 = new float[128 * descriptors[img2Id].size()];
	for (unsigned int i = 0; i < descriptors[img1Id].size(); i++)
		std::copy(descriptors[img1Id][i].begin(), descriptors[img1Id][i].end(), data1 + 128 * i);
	for (unsigned int i = 0; i < descriptors[img2Id].size(); i++)
		std::copy(descriptors[img2Id][i].begin(), descriptors[img2Id][i].end(), data2 + 128 * i);
	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);
	cout << "descriptors[img1Id].size()" << descriptors[img1Id].size() << endl;
	vl_kdforest_build(forest, descriptors[img1Id].size(), data1);

	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	VlKDForestNeighbor neighbours[2];

	//vector<pair<int, int>> matchedPairsVec;
	matchedPairsVec.clear();
	// testing
	/*CImg<unsigned char> result = CImg<unsigned char>(imgs[img1Id].width() + imgs[img2Id].width(), imgs[img1Id].height(), 1, 3);
	cimg_forXY(imgs[img1Id], x, y) {
		result(x, y, 0, 0) = imgs[img1Id](x,y,0,0);
		result(x, y, 0, 1) = imgs[img1Id](x, y, 0, 1);
		result(x, y, 0, 2) = imgs[img1Id](x, y, 0, 2);
		result(x + imgs[img1Id].width(), y, 0, 0) = imgs[img2Id](x, y, 0, 0);
		result(x + imgs[img1Id].width(), y, 0, 1) = imgs[img2Id](x, y, 0, 1);
		result(x + imgs[img1Id].width(), y, 0, 2) = imgs[img2Id](x, y, 0, 2);
	}*/
	// testing end
	for (int i = 0; i < descriptors[img2Id].size(); i++) {
		vl_kdforestsearcher_query(searcher, neighbours, 2, data2+128*i);
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		int color[3] = { r,g,b };
		if (neighbours[0].distance < 0.8*neighbours[1].distance) {
			// todo ��ƥ������Դ�������
			matchedPairsVec.push_back(make_pair(neighbours[0].index, i));
			// testing
			//result.draw_circle(keyPoints[img1Id][neighbours[0].index].x, keyPoints[img1Id][neighbours[0].index].y, 1, color);
			//result.draw_circle(keyPoints[img2Id][i].x + imgs[img1Id].width(), keyPoints[img2Id][i].y, 1, color);
			//result.draw_line(keyPoints[img1Id][neighbours[0].index].x, keyPoints[img1Id][neighbours[0].index].y, keyPoints[img2Id][i].x + imgs[img1Id].width(), keyPoints[img2Id][i].y, color);
			// testing end
		}
	}

	// result.display();
}

void Pano::warping(Matrix HMat, int img1Id, int img2Id) {
	// testing
	CImg<unsigned char> result = CImg<unsigned char>(imgs[img1Id].width() + imgs[img2Id].width(), imgs[img1Id].height(), 1, 3);
	cimg_forXY(imgs[img1Id], x, y) {
		result(x, y, 0, 0) = imgs[img1Id](x, y, 0, 0);
		result(x, y, 0, 1) = imgs[img1Id](x, y, 0, 1);
		result(x, y, 0, 2) = imgs[img1Id](x, y, 0, 2);
		result(x + imgs[img1Id].width(), y, 0, 0) = imgs[img2Id](x, y, 0, 0);
		result(x + imgs[img1Id].width(), y, 0, 1) = imgs[img2Id](x, y, 0, 1);
		result(x + imgs[img1Id].width(), y, 0, 2) = imgs[img2Id](x, y, 0, 2);
	}
	// testing end
	cimg_forXY(imgs[img2Id], x, y) {
		if (imgs[img2Id](x, y, 0, 0) != 0 || imgs[img2Id](x, y, 0, 1) != 0 || imgs[img2Id](x, y, 0, 2) != 0) {
			// ����任����±�
		// ��ǰӳ��
			double data[3] = { x, y, 1 };
			Matrix xy(data, 3, 1);
			Matrix xyPrime = HMat * xy;
			int xPrime = round(xyPrime.item[0]);
			int yPrime = round(xyPrime.item[1]);
			if (xPrime < result.width() && xPrime >= 0 && yPrime >= 0 && yPrime < result.height()) {
				result(xPrime, yPrime, 0, 0) = imgs[img2Id](x, y, 0, 0);
				result(xPrime, yPrime, 0, 1) = imgs[img2Id](x, y, 0, 1);
				result(xPrime, yPrime, 0, 2) = imgs[img2Id](x, y, 0, 2);
			}
		}
	}
	result.display();
	result.save("temp.png");

}