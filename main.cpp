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
#define N 16

int main(int argc, _TCHAR* argv[])
{
	int n = N;
	
	string filenames[N] = { "sph1.bmp", "sph2.bmp", "sph3.bmp", "sph4.bmp", "sph5.bmp", "sph6.bmp", "sph7.bmp", "sph8.bmp",
		"sph9.bmp", "sph10.bmp", "sph11.bmp", "sph12.bmp", "sph13.bmp", "sph14.bmp", "sph15.bmp", "sph16.bmp"};
	/*string filenames[N] = { "1.bmp", "2.bmp", "3.bmp", "4.bmp", "5.bmp", "6.bmp", "7.bmp", "8.bmp",
		"9.bmp", "10.bmp", "11.bmp", "12.bmp", "13.bmp", "14.bmp", "15.bmp", "16.bmp"};*/
	// 读入所有全景图片
	Pano pano(filenames , N);
	return 0;
}