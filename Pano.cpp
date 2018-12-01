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
	num = n + 1;
	// 中间两图的坐标
	int img1Id = num / 2, img2Id = img1Id + 1;
	imgs.push_back(resultImage);
	for (int i = 0; i < n; i++) {
		CImg<unsigned char> tempImg(filenames[i].c_str());
		imgs.push_back(tempImg);
	}
	// 进行球坐标变换
	SphericalTrans st(imgs, num);
	resultImage = CImg<unsigned char>(imgs[img1Id].width() * (num-1)/1.8, imgs[img1Id].height(), 1, 3);
	
	
	// 初始化resultImage
	cimg_forXY(resultImage, x, y) {
		resultImage(x, y, 0, 0) = 0;
		resultImage(x, y, 0, 1) = 0;
		resultImage(x, y, 0, 2) = 0;
	}
	cout << "imgs[img1Id].width() " << imgs[img1Id].width() << " img1Id " << img1Id << endl;
	cimg_forXY(imgs[img1Id], x, y) {
		resultImage(x + imgs[img1Id].width() * (img1Id - 1)/2, y, 0, 0) = imgs[img1Id](x, y, 0, 0);
		resultImage(x + imgs[img1Id].width() * (img1Id - 1)/2, y, 0, 1) = imgs[img1Id](x, y, 0, 1);
		resultImage(x + imgs[img1Id].width() * (img1Id - 1)/2, y, 0, 2) = imgs[img1Id](x, y, 0, 2);
	}
	left[img1Id] = imgs[img1Id].width() * (img1Id - 1) / 2;
	right[img1Id] = left[img1Id] + imgs[img1Id].width();
	imgs[0] = resultImage; // 第0个图片为result
	getFeatures(); // 获取所有图片的特征点
	doPairsMatching(0, img2Id);
	img1Id--; img2Id++;
	for (int t = 0; t < num / 2; t++) {
		cout << img1Id << " " << img2Id << endl;
		if (img1Id >= 1) {
			updateFeatures(img1Id);
			doPairsMatching(0, img1Id);
			img1Id--;
		}
		if (img2Id <= num - 1) {
			updateFeatures(img2Id);
			doPairsMatching(0, img2Id);
			img2Id++;
		}		
	}

}

Pano::~Pano()
{
}

void Pano::doPairsMatching(int zero, int imgId) {
	getMatchedPairs(imgId);
	Ransac ransac(matchedPairsVec, keyPoints, descriptors, 0, imgId, imgs);
	Matrix HMat(*(ransac.getBestH()));
	warping(HMat, 0, imgId);
}

void Pano::getFeatures() {
	cout << "get features of all the pics" << endl;
	VlSiftFilt *SiftFilt = NULL;
	// 此步骤仍然所有图片尺寸一致
	
	for (int k = 0; k < num; k++) {
		cout << "features in " << k << " start" << endl;
		// 新建所有图片的像素一维矩阵
		//int noctaves = log(sqrt(imgs[1].height()*imgs[1].width())) / log(2) - 3;
		int noctaves = 4;
		int nlevels = 2, o_min = 0; // octave数目，每个octave层级数目
		SiftFilt = vl_sift_new(imgs[k].width(), imgs[k].height(), noctaves, nlevels, o_min);
		vl_sift_pix *ImageData = new vl_sift_pix[imgs[k].height()*imgs[k].width()];
		for (int i = 0; i < imgs[k].height(); i++) {
			for (int j = 0; j < imgs[k].width(); j++) {
				int gray = imgs[k](j, i, 0, 0) * 0.299 + imgs[k](j, i, 0, 1) * 0.587 + imgs[k](j, i, 0, 2) * 0.114;
				ImageData[i*imgs[k].width() + j] = gray;
			}
		}
		vector< VlSiftKeypoint > tempKeys; 
		vector< vector <float> > tempDes; // 当前图片描述子序列
		if (vl_sift_process_first_octave(SiftFilt, ImageData) != VL_ERR_EOF) {
			while (1){
				//计算每组中的关键点
				vl_sift_detect(SiftFilt);
				VlSiftKeypoint *pKeyPoint = SiftFilt->keys; // 特征点序列
				 // 生成描述子序列
				for (int i = 0; i < SiftFilt->nkeys; i++) { // 遍历每一个特征点
					VlSiftKeypoint TemptKeyPoint = *pKeyPoint; // 当前特征点
					pKeyPoint++;
					//计算并遍历每个点的方向
					double angles[4];
					int angleCount = vl_sift_calc_keypoint_orientations(SiftFilt, angles, &TemptKeyPoint); // 最多四个
					for (int j = 0; j < angleCount; j++)  {// 遍历所有的方向
						tempKeys.push_back(TemptKeyPoint);
						double TemptAngle = angles[j];
						//计算每个方向的描述
						float *Descriptors = new float[128];
						vl_sift_calc_keypoint_descriptor(SiftFilt, Descriptors, &TemptKeyPoint, TemptAngle);// 获得当前方向的描述子
						// 存入描述子序列
						vector<float> copyOfDes;
						for (int p = 0; p < 128; p++) {
							copyOfDes.push_back(*(Descriptors + p));
						}
						tempDes.push_back(copyOfDes);
						delete[] Descriptors;
						Descriptors = NULL;
					}
				}
				//下一阶
				if (vl_sift_process_next_octave(SiftFilt) == VL_ERR_EOF)
				{
					break;
				}
			}

		}
		delete[] ImageData;
		vl_sift_delete(SiftFilt);
		descriptors.push_back(tempDes);
		keyPoints.push_back(tempKeys);
	}
}

void Pano::updateFeatures(int imgId) {
	cout << "update features of all the pics" << endl;
	VlSiftFilt *SiftFilt = NULL;
	int referId = imgId < num / 2 ? imgId + 1 : imgId - 1;
	CImg<unsigned char> * test = new CImg<unsigned char>(right[referId] - left[referId], imgs[0].height(), 1,3);
	cimg_forXY(*test, x, y) {
		(*test)(x, y, 0, 0) = imgs[0](x + left[referId],y,0,0);
		(*test)(x, y, 0, 1) = imgs[0](x + left[referId], y, 0, 1);
		(*test)(x, y, 0, 2) = imgs[0](x + left[referId], y, 0, 2);
	}
	// int noctaves = log(sqrt(imgs[1].height()*imgs[1].width())) / log(2) - 3;
	int noctaves = 4;
	int nlevels = 2, o_min = 0; // octave数目，每个octave层级数目
	SiftFilt = vl_sift_new((*test).width(), (*test).height(), noctaves, nlevels, o_min);
	cout << "right[referId] " << right[referId] << "left[referId]) " << left[referId] << "refer " << referId << endl;
	vl_sift_pix *ImageData = new vl_sift_pix[(*test).height()*(*test).width()/**imgs[0].width()*/];
	for (int i = 0; i < imgs[0].height(); i++) {
		for (int j = 0; j < (*test).width(); j++) {
			int gray = (*test)(j, i, 0, 0) * 0.299 + (*test)(j, i, 0, 1) * 0.587 + (*test)(j, i, 0, 2) * 0.114;
			ImageData[i*((*test).width()) + j] = gray;
		}
	}
	vector< VlSiftKeypoint > tempKeys;
	vector< vector <float> > tempDes; // 当前图片描述子序列
	if (vl_sift_process_first_octave(SiftFilt, ImageData) != VL_ERR_EOF) {
		while (1) {
			//计算每组中的关键点
			vl_sift_detect(SiftFilt);
			VlSiftKeypoint *pKeyPoint = SiftFilt->keys; // 特征点序列
			cout << "特征点数目 " << SiftFilt->nkeys << endl;
				// 生成描述子序列
			for (int i = 0; i < SiftFilt->nkeys; i++) { // 遍历每一个特征点
				VlSiftKeypoint TemptKeyPoint = *pKeyPoint; // 当前特征点
				pKeyPoint++;
				//计算并遍历每个点的方向
				double angles[4];
				int angleCount = vl_sift_calc_keypoint_orientations(SiftFilt, angles, &TemptKeyPoint); // 最多四个
				for (int j = 0; j < angleCount; j++) {// 遍历所有的方向
					VlSiftKeypoint temp = TemptKeyPoint;
					temp.x += left[referId];
					tempKeys.push_back(temp);
					double TemptAngle = angles[j];
					//计算每个方向的描述
					float *Descriptors = new float[128];
					vl_sift_calc_keypoint_descriptor(SiftFilt, Descriptors, &TemptKeyPoint, TemptAngle);// 获得当前方向的描述子
					// 存入描述子序列
					vector<float> copyOfDes;
					for (int p = 0; p < 128; p++) {
						copyOfDes.push_back(*(Descriptors+p));
					}
					tempDes.push_back(copyOfDes);
					delete [] Descriptors;
					Descriptors = NULL;
				}
			}
			//下一阶
			if (vl_sift_process_next_octave(SiftFilt) == VL_ERR_EOF) {
				break;
			}
		}

	}
	vl_sift_delete(SiftFilt);
	delete[] ImageData;
	descriptors[0] = (tempDes);
	keyPoints[0] = (tempKeys);
	delete test;
}

void Pano::getMatchedPairs(int imgId) {
	float* data1;
	float* data2;
	// 将左图中的数据存储起来
	data1 = new float[128 * descriptors[0].size()];
	data2 = new float[128 * descriptors[imgId].size()];
	for (unsigned int i = 0; i < descriptors[0].size(); i++)
		std::copy(descriptors[0][i].begin(), descriptors[0][i].end(), data1 + 128 * i);
	for (unsigned int i = 0; i < descriptors[imgId].size(); i++)
		std::copy(descriptors[imgId][i].begin(), descriptors[imgId][i].end(), data2 + 128 * i);
	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);
	cout << " descriptors[0].size()  " << descriptors[0].size() << endl;
	vl_kdforest_build(forest, descriptors[0].size(), data1);

	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	VlKDForestNeighbor neighbours[2];
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
	for (int i = 0; i < descriptors[imgId].size(); i++) {
		vl_kdforestsearcher_query(searcher, neighbours, 2, data2+128*i);
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		int color[3] = { r,g,b };
		if (neighbours[0].distance < 0.8*neighbours[1].distance) {
			matchedPairsVec.push_back(make_pair(neighbours[0].index, i));
		}
	}
	vl_kdforestsearcher_delete(searcher);
	vl_kdforest_delete(forest);
	delete[] data1;
	delete[] data2;
	cout << "配对点的数目" << matchedPairsVec.size() << endl;
	
	
}

void Pano::warping(Matrix HMat, int img1Id, int img2Id) {
	//cout << "start warping img2" << endl;
	// 根据在左边和右边判断alpha
	cout << "imgid" << img2Id << endl;
	double temp1[3] = { 0,0,1 }; // 左上
	Matrix lt(temp1, 3, 1);
	double temp2[3] = { imgs[img2Id].width(), 0, 1 }; // 右上
	Matrix rt(temp2, 3, 1);
	double temp3[3] = {0, imgs[img2Id].height(), 1};
	Matrix lb(temp3, 3, 1); // 左下
	double temp4[3] = { imgs[img2Id].width(), imgs[img2Id].height(), 1 };
	Matrix rb(temp4, 3, 1); // 右下
	cout << HMat << endl;
	int lowY = min((HMat*lt).item[1], (HMat*rt).item[1]) >= 0? min((HMat*lt).item[1], (HMat*rt).item[1]):0;
	int highY = max((HMat*lb).item[1],(HMat*rb).item[1]) < imgs[0].height() ? max((HMat*lb).item[1], (HMat*rb).item[1]) : imgs[0].height()-1;
	int lowX = min((HMat*lt).item[0], (HMat*lb).item[0]) >= 0 ? min((HMat*lt).item[0], (HMat*lb).item[0]) : 0;
	int highX = max((HMat*rt).item[0], (HMat*rb).item[0]) < imgs[0].width() ? max((HMat*rt).item[0], (HMat*rb).item[0]) : imgs[0].width() - 1;
	left[img2Id] = lowX;
	right[img2Id] = highX;
	cout << lowX << " ------ " << highX << " " << lowY << " " << highY << endl;
	Matrix inverse(HMat.Inverse());
	//cimg_forXY(imgs[0],x,y) {
	for (int x = lowX; x < highX; x++) {
		for (int y = lowY; y < highY; y++) {
			double b[3] = { x, y, 1 };
			Matrix xy(b, 3, 1);
			Matrix xyPrime = inverse * xy;
			int xPrime = round(xyPrime.item[0]);
			int yPrime = round(xyPrime.item[1]);
			if (xPrime < imgs[img2Id].width() && xPrime >= 0 && yPrime >= 0 && yPrime < imgs[img2Id].height()) {
				// 如果当前位置为黑色，则全部为新图片
				if (imgs[0](x, y, 0, 0) == 0 && imgs[0](x, y, 0, 1) == 0 && imgs[0](x, y, 0, 2) == 0) {
					imgs[0](x, y, 0, 0) = imgs[img2Id](xPrime, yPrime, 0, 0);
					imgs[0](x, y, 0, 1) = imgs[img2Id](xPrime, yPrime, 0, 1);
					imgs[0](x, y, 0, 2) = imgs[img2Id](xPrime, yPrime, 0, 2);
				}
				// 如果当前位置原图有内容，则融合
				else if (imgs[img2Id](xPrime, yPrime, 0, 0) != 0 && imgs[img2Id](xPrime, yPrime, 0, 1) != 0 && imgs[img2Id](xPrime, yPrime, 0, 2) != 0) {
					double alpha = 1;
					if (img2Id < num / 2) { // 在左侧，x越小 alpha越大
						alpha = 1 - xPrime / imgs[img2Id].width();
					}
					else {
						alpha = xPrime / imgs[img2Id].width();
					}
					imgs[0](x, y, 0, 0) = (imgs[img2Id](xPrime, yPrime, 0, 0)*alpha + imgs[0](x, y, 0, 0)) / (alpha + 1);
					imgs[0](x, y, 0, 1) = (imgs[img2Id](xPrime, yPrime, 0, 1)*alpha + imgs[0](x, y, 0, 1)) / (alpha + 1);
					imgs[0](x, y, 0, 2) = (imgs[img2Id](xPrime, yPrime, 0, 2)*alpha + imgs[0](x, y, 0, 2)) / (alpha + 1);
				}
			}
		}
	}
	imgs[0].save("temp2.png");
}