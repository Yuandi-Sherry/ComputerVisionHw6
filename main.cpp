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

int main(int argc, _TCHAR* argv[])
{
	string filenames[4] = { "1.bmp", "2.bmp", "3.bmp", "4.bmp" };
	// 读入所有全景图片
	Pano pano(filenames ,4);
	return 0;
}