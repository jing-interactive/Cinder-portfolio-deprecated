#include "stdafx.h"
#include "OpenCVExtention.h"
#include <stdio.h>
#include <string>
#include <Windows.h>

TCHAR* StringToTCHAR(string str)
{
	size_t len = strlen(str.c_str());
	size_t wlen = MultiByteToWideChar(CP_ACP, 0, (const char*)str.c_str(), int(len), NULL, 0);
	TCHAR *wBuf = new TCHAR[wlen + 1];
	MultiByteToWideChar(CP_ACP, 0, (const char*)str.c_str(), int(len), wBuf, int(wlen));
	wBuf[ wlen ] = _T('\0');
	return wBuf;
}

string TCHARToString(TCHAR * tchar)
{
    char cBuf[127];
	WideCharToMultiByte(   CP_ACP,   WC_COMPOSITECHECK,   tchar,   -1,   cBuf,   sizeof(cBuf),   NULL,   NULL   );   
    string str(cBuf);
	return str;
}

IplImage* wcvLoadImage(const wchar_t* filename, int iscolor)
{
	assert(filename);

	int len = (int) wcslen(filename) + 1;
	len *= 2;

	char* buf = new char[len];

	WideCharToMultiByte(CP_ACP, 0, filename, -1, buf, len, 0, 0);

	IplImage* img = 0;
	img = cvLoadImage(buf, iscolor);

	delete []buf;

	return img;
}

int wcvSaveImage(const wchar_t* filename, const CvArr* image, const int* params)
{
	assert(filename);

	int len = (int) wcslen(filename) + 1;
	len *= 2;

	char* buf = new char[len];

	WideCharToMultiByte(CP_ACP, 0, filename, -1, buf, len, 0, 0);

	int ret;
	ret = cvSaveImage(buf, image);

	delete []buf;

	return ret;
}

#if (CV_MAJOR_VERSION == 1 && CV_MINOR_VERSION == 0)
static bool _getSubSetForHomographyEstimation(CvMat *m1, CvMat *m2, CvMat *ms1, CvMat *ms2, int count, int maxAttemptNum, CvRNG *rng)
{
	ASSERT_CVMAT_EQUAL_FORMAT(m1, m2);
	int ptNum = m1->rows; 

	int *idx = new int[count];
	assert(idx);

	int iter, i, j;
	for(iter = 0; iter < maxAttemptNum; iter++){
		for(i = 0; i < count && iter < maxAttemptNum; ){
			idx[i] = cvRandInt(rng) % ptNum;
			//idx[i] = rand() % ptNum;
			for(j = 0; j < i; j++){
				if(idx[j] == idx[i]) break;
			}
			if(j < i) continue;

			CV_MAT_ELEM(*ms1, float, i, 0) = CV_MAT_ELEM(*m1, float, idx[i], 0);
			CV_MAT_ELEM(*ms1, float, i, 1) = CV_MAT_ELEM(*m1, float, idx[i], 1);
			CV_MAT_ELEM(*ms1, float, i, 2) = 1.f;

			CV_MAT_ELEM(*ms2, float, i, 0) = CV_MAT_ELEM(*m2, float, idx[i], 0);
			CV_MAT_ELEM(*ms2, float, i, 1) = CV_MAT_ELEM(*m2, float, idx[i], 1);
			CV_MAT_ELEM(*ms2, float, i, 2) = 1.f;
			i++;
			//TODO::
			//check co-line situation
		}
		break;
	}

	delete []idx;
	if(i == count) return true;
	else return false;
}

static int _findInlinerForHomographyEstimation(CvMat *srcPts, CvMat *dstPts, CvMat *HMat, CvMat *mask, float inlierThreshold)
{
	cvZero(mask);
	int ptNum = srcPts->rows;
	inlierThreshold *= inlierThreshold;

	float H[9];
	H[0] = CV_MAT_ELEM(*HMat, float, 0, 0);
	H[1] = CV_MAT_ELEM(*HMat, float, 0, 1);
	H[2] = CV_MAT_ELEM(*HMat, float, 0, 2);
	H[3] = CV_MAT_ELEM(*HMat, float, 1, 0);
	H[4] = CV_MAT_ELEM(*HMat, float, 1, 1);
	H[5] = CV_MAT_ELEM(*HMat, float, 1, 2);
	H[6] = CV_MAT_ELEM(*HMat, float, 2, 0);
	H[7] = CV_MAT_ELEM(*HMat, float, 2, 1);
	H[8] = 1.f;

	float errorX, errorY;
	int count = 0;
	for(int i = 0; i < ptNum; i++){
		float *srcPtr = (float*) (srcPts->data.ptr + srcPts->step*i);
		float *dstPtr = (float*) (dstPts->data.ptr + dstPts->step*i);
		errorX = srcPtr[0]*H[0] + srcPtr[1]*H[1] + H[2] - dstPtr[0];
		errorY = srcPtr[0]*H[3] + srcPtr[1]*H[4] + H[5] - dstPtr[1];
		if(errorX*errorX + errorY*errorY < inlierThreshold){
			CV_MAT_ELEM(*mask, unsigned char, 0, i) = 1;
			count++;
		}
	}
	return count;
}

static void _refineHomographyEstimation(CvMat *srcPts, CvMat *dstPts, CvMat *H, CvMat *mask, int goodCount)
{
	CvMat *srcInlier = cvCreateMat(goodCount, 3, CV_32F);
	CvMat *dstInlier = cvCreateMat(goodCount, 3, CV_32F);
	assert(srcInlier);
	assert(dstInlier);

	float *ptr1, *ptr2;
	int count = 0;
	for(int i = 0; i < mask->rows; i++){
		if(CV_MAT_ELEM(*mask, unsigned char, 0, i) != 0){
			ptr1 = (float*) (srcPts->data.ptr + srcPts->step*i);
			ptr2 = (float*) (srcInlier->data.ptr + srcInlier->step*count);
			memcpy(ptr2, ptr1, 3*sizeof(float));

			ptr1 = (float*) (dstPts->data.ptr + dstPts->step*i);
			ptr2 = (float*) (dstInlier->data.ptr + dstInlier->step*count);
			memcpy(ptr2, ptr1, 3*sizeof(float));
			
			count++;
		}
	}
	
	assert(count == goodCount);
	cvFindHomography(srcInlier, dstInlier, H);

	cvReleaseMat(&srcInlier);
	cvReleaseMat(&dstInlier);
}

int _findHomographyRansac(CvMat *srcPts, CvMat *dstPts, CvMat *H, float inlierThreshold, CvMat* mask)
{
	ASSERT_CVMAT_EQUAL_FORMAT(srcPts, dstPts);
	assert(H->width == 3 && H->height == 3);
	assert(srcPts->height >= 4);
	assert(mask->rows == 1);

	int ptNum = srcPts->rows;
	int iterNumMax = 2000;
	int iter = 0;
	
	int goodCount;
	int goodCountMax = 0;

	CvMat *ms1 = cvCreateMat(4, 3, CV_32FC1);
	CvMat *ms2 = cvCreateMat(4, 3, CV_32FC1);
	assert(ms1);
	assert(ms2);

	//CvMat *mask = cvCreateMat(1,ptNum, CV_8U);
	CvMat *maskFinal = cvCloneMat(mask);
	assert(mask);
	assert(maskFinal);

	CvRNG rng = 13;
	for(iter = 0; iter < iterNumMax; iter++){
		bool found = _getSubSetForHomographyEstimation(srcPts, dstPts, ms1, ms2, 4, 300, &rng);
		if(!found){
			if( iter == 0 )
				return false;
			break;
		}

		//call the built-in function to make things simple.
		cvFindHomography(ms1, ms2, H);

		cvZero(mask);
		goodCount = _findInlinerForHomographyEstimation(srcPts, dstPts, H, mask, inlierThreshold);
		if(goodCount > goodCountMax){
			goodCountMax = goodCount;
			cvCopy(mask, maskFinal);
		}

		//TODO:
		//update iterNumMax
	}
	
	printf("goodCountMax = %d\n", goodCountMax);
	//refine result.
	_refineHomographyEstimation(srcPts, dstPts, H, maskFinal, goodCountMax);
	
	cvReleaseMat(&ms1);
	cvReleaseMat(&ms2);
	cvReleaseMat(&mask);
	cvReleaseMat(&maskFinal);

	return true;
}
#endif

int findHomographyRansac(CvMat *srcPts, CvMat *dstPts, CvMat *H, float inlierThreshold, CvMat *mask)
{
#if (CV_MAJOR_VERSION == 1 && CV_MINOR_VERSION == 0)
	return _findHomographyRansac(srcPts, dstPts, H, inlierThreshold, mask);
#elif ( (CV_MAJOR_VERSION == 1 && CV_MINOR_VERSION == 1) || CV_MAJOR_VERSION == 2 )
	return cvFindHomography(srcPts, dstPts, H,CV_RANSAC, inlierThreshold, mask);
#else
#Pragma message(¡°Error: Unknown OpenCV Version¡±)
	assert(0);
#endif
}

void cvShowImageSmart(const char* name, const CvArr* image)
{
	CvMat stub, *img;
#ifdef _WIN32
	int   screenFullWidth  = GetSystemMetrics(SM_CXSCREEN);   
	int   screenFullHeight = GetSystemMetrics(SM_CYSCREEN);
	img = cvGetMat(image, &stub);
	if(img->width > screenFullWidth || img->height > screenFullHeight){
		float ratio = MIN(screenFullWidth / (float) img->width, screenFullHeight / (float) img->height);
		int newWidth = (int) (img->width * ratio);
		int newHeight = (int) (img->height * ratio);
		CvMat* newImg = cvCreateMat(newHeight, newWidth, img->type);
		cvResize(img, newImg);
		cvShowImage(name, newImg);
		cvReleaseMat(&newImg);
	}else{
		cvShowImage(name, image);
	}
#else
	cvShowImage(name, image);
	return;
#endif
}
