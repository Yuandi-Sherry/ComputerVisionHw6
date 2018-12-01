#include "SphericalTrans.h"
#include "CImg.h"
#include <string>
using namespace std;
using namespace cimg_library;

SphericalTrans::SphericalTrans(CImg<unsigned char> * imgs, int n)
{
	focus = 595;
	for (int i = 0; i < n; i++) {
		// imgs[i].display();
		CImg<unsigned char> temp(imgs[i].width(), imgs[i].height(), 1, 3);
		cimg_forXY(temp, x, y) {
			temp(x, y, 0, 0) = 0;
			temp(x, y, 0, 1) = 0;
			temp(x, y, 0, 2) = 0;
		}
		cimg_forXY(imgs[i], x, y) {
			int xCenter = imgs[i].width() / 2;
			int yCenter = imgs[i].height() / 2;
			int X = x - xCenter;
			int Y = y - yCenter;
			double R = sqrt(pow(X, 2) + pow(Y, 2) + pow(focus, 2)); 
			double xHat = X / R;
			double yHat = Y / R;
			double phi = asin(yHat);
			double theta = asin(xHat / cos(phi));
			int xTide = focus * theta + xCenter;
			int yTide = focus * phi + yCenter;
			if (xTide < temp.width() && xTide >= 0 && yTide < temp.height() && yTide >= 0) {
				temp(xTide, yTide, 0, 0) = imgs[i](x, y, 0, 0);
				temp(xTide, yTide, 0, 1) = imgs[i](x, y, 0, 1);
				temp(xTide, yTide, 0, 2) = imgs[i](x, y, 0, 2);
			}
		}
		imgs[i] = temp;
		string name = "sph" + to_string(i);
		imgs[i].save((name + ".png").c_str());
	}
	
}

SphericalTrans::~SphericalTrans()
{
}