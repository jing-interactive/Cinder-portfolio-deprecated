#ifndef __OPENCV_EXTENTION_H__
#define __OPENCV_EXTENTION_H__

//这里声明的opencv扩展函数完全独立，不与其他任何代码耦合。
//#include "cxcore.h"
//#include "cv.h"
//#include "highgui.h"
#include <tchar.h>
#include <opencv2/opencv.hpp>
#include <string>
using namespace std;

//macros for check arguments
#define ASSERT_IPLIMAGE_EQUAL_SIZE(img1, img2) assert(img1->width == img2->width && img1->height == img2->height)
#define ASSERT_IPLIMAGE_EQUAL_CHANNEL(img1, img2) assert(img1->nChannels == img2->nChannels)
#define ASSERT_IPLIMAGE_EQUAL_DEPTH(img1, img2) assert(img1->depth == img2->depth)
#define ASSERT_IPLIMAGE_EQUAL_FORMAT(img1, img2) assert(img1->width == img2->width &&\
	img1->height == img2->height &&\
	img1->depth == img2->depth &&\
	img1->nChannels == img2->nChannels)

#define ASSERT_IPLIMAGE_IS_GRAY(img) assert(img->depth = 8 && img->nChannels == 1)
#define ASSERT_IPLIMAGE_IS_RGB(img) assert(img->depth = 8 && img->nChannels == 3)

#define ASSERT_CVMAT_EQUAL_SIZE(mat1, mat2) assert(mat1->width == mat2->width && mat1->height == mat2->height)
#define ASSERT_CVMAT_EQUAL_TYPE(mat1, mat2) assert(mat1->type == mat2->type)
#define ASSERT_CVMAT_EQUAL_FORMAT(mat1, mat2) assert(mat1->width == mat2->width && mat1->height == mat2->height && mat1->type == mat2->type)


//cvLoadImage, cvSaveImage unicode version
IplImage* wcvLoadImage(const wchar_t* filename, int iscolor CV_DEFAULT(CV_LOAD_IMAGE_COLOR));
int wcvSaveImage(const wchar_t* filename, const CvArr* image, const int* params CV_DEFAULT(0));
#ifdef _UNICODE
#define _tcvLoadImage wcvLoadImage 
#define _tcvSaveImage wcvSaveImage 
#else
#define  _tcvLoadImage cvLoadImage
#define _tcvSaveImage cvSaveImage
#endif

//homography estimator.
//相当于实现opencv2.0里面的cvFindHomography
//srcPts, dstPts应是N*3的矩阵
//mask应是1*N的矩阵
int findHomographyRansac(CvMat *srcPts, CvMat *dstPts, CvMat *H, float inlierThreshold, CvMat *mask);

void cvShowImageSmart(const char* name, const CvArr* image);

TCHAR* StringToTCHAR(string str);
string TCHARToString(TCHAR * tchar);

#endif