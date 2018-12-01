#include "CImg.h"

using namespace cimg_library;
#ifndef _SPHERICALTRANS_H_
#define _SPHERICALTRANS_H_
class SphericalTrans
{
public:
	SphericalTrans(CImg<unsigned char> * imgs, int n);
	~SphericalTrans();

private:
	int focus;
};


#endif // !_SPHERICALTRANS

