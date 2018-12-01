#include "CImg.h"
#include <vector>
using namespace std;
using namespace cimg_library;
#ifndef _SPHERICALTRANS_H_
#define _SPHERICALTRANS_H_
class SphericalTrans
{
public:
	SphericalTrans(vector< CImg<unsigned char> > & imgs, int n);
	~SphericalTrans();

private:
	int focus;
};


#endif // !_SPHERICALTRANS

