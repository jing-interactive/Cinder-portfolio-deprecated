#pragma once

#include <OpenCV/cv.h>
#include <vector>
#include <string>

using std::vector; 
using std::string;

#define DIST_THRESH 1025021

struct profile_t
{//一个档案，不一定是人
	char name[128];
	//vector<IplImage*> photos;
	IplImage* photo;
};


struct recognition_t
{//识别
	vector<profile_t> profile_vec; //训练用，用于映射
	vector<IplImage*> train_images;//训练用，用于图片序列

	int nTrainFaces; 
	int nEigens; // the number of eigenvalues
	IplImage * pAvgTrainImg; // the average image
	IplImage ** eigenVectArr; // eigenvectors
	CvMat * eigenValMat; // eigenvalues
	CvMat * projectedTrainFaceMat; // projected training faces

	~recognition_t()
	{
		release();
	}

	recognition_t()
	{   
		nTrainFaces = 0;
		nEigens = 0;
		pAvgTrainImg = NULL;
		eigenVectArr = NULL; 
		eigenValMat = NULL;
		projectedTrainFaceMat = NULL;
	}

	void release()
	{
		int n = train_images.size();
		for (int i =0;i<n; i++)
			cvReleaseImage(&train_images[i]);
		train_images.clear();
		profile_vec.clear();

		if (pAvgTrainImg)
			cvReleaseImage(&pAvgTrainImg);

		if (eigenVectArr)
		{			
			for (int i=0;i<nEigens;i++)
				cvReleaseImage(&eigenVectArr[i]);
			cvFree(&eigenVectArr);
		}

		if (eigenValMat)
			cvReleaseMat(&eigenValMat);

		if (projectedTrainFaceMat)
			cvReleaseMat(&projectedTrainFaceMat);

	}

	void add_profile(IplImage* image, const char* object_name)
	{//增加档案，供bool learn()学习用
		profile_t prf;
		prf.photo = image;
		strcpy(prf.name, object_name);

		profile_vec.push_back(prf);

#ifdef TOP_HAT
		cvMorphologyEx(image, image, 0, 0, CV_MOP_TOPHAT);
#endif

		train_images.push_back(image);
	}

	bool learn()
	{
		//对profile_vec中的数据进行学习
		//[do PCA]
		int i, offset; 

		nTrainFaces = profile_vec.size();
		if (nTrainFaces < 2)
			return false;

		// set the number of eigenvalues to use
		nEigens = nTrainFaces-1;

		// allocate the eigenvector images
		CvSize faceImgSize = cvGetSize(train_images[0]);
		//faceImgSize.width  = profile_vec[0].photo->width;
		//faceImgSize.height = profile_vec[0].photo->height;
		eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);
		for(i=0; i<nEigens; i++)
			eigenVectArr[i] = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

		// allocate the eigenvalue array
		eigenValMat = cvCreateMat( 1, nEigens, CV_32FC1 );

		// allocate the averaged image
		pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

		// set the PCA termination criterion
		CvTermCriteria calcLimit = cvTermCriteria( CV_TERMCRIT_ITER, nEigens, 1);

		// compute average image, eigenvalues, and eigenvectors
		cvCalcEigenObjects(
			nTrainFaces,
			(void*)(&train_images[0]),
			(void*)eigenVectArr,
			CV_EIGOBJ_NO_CALLBACK,
			0,
			0,
			&calcLimit,
			pAvgTrainImg,
			eigenValMat->data.fl);

		cvNormalize(eigenValMat, eigenValMat, 1, 0, CV_L1, 0);

		// project the training images onto the PCA subspace
		projectedTrainFaceMat = cvCreateMat( nTrainFaces, nEigens, CV_32FC1 );
		offset = projectedTrainFaceMat->step / sizeof(float);
		for(i=0; i<nTrainFaces; i++)
		{
			//int offset = i * nEigens;
			cvEigenDecomposite(
				train_images[i],
				nEigens,
				eigenVectArr,
				0, 0,
				pAvgTrainImg,
				//projectedTrainFaceMat->data.fl + i*nEigens);
				projectedTrainFaceMat->data.fl + i*offset);
		}

		return true;
	}

	char* recognize(IplImage* image)//return the name
	{
		float * projected = 0;

		// project the test images onto the PCA subspace
		projected = (float *)cvAlloc( nEigens*sizeof(float) );

#ifdef TOP_HAT
		cvMorphologyEx(image, image, 0, 0, CV_MOP_TOPHAT);
#endif

		// project the test image onto the PCA subspace
		cvEigenDecomposite(
			image,
			nEigens,
			eigenVectArr,
			0, 0,
			pAvgTrainImg,
			projected);

		int iNearest = findNearestNeighbor(projected); 

		char* result = "stranger";
		if (iNearest != -1)
			result  = profile_vec[iNearest].name;

		cvFree(&projected);

		return result;
	}

	//////////////////////////////////
	// findNearestNeighbor()
	//
	int findNearestNeighbor(float * projectedTestFace)
	{
		//double leastDistSq = 1e12;
		double leastDistSq = DBL_MAX;
		int i, iTrain, iNearest = 0;

		for(iTrain=0; iTrain<nTrainFaces; iTrain++)
		{
			double distSq=0;

			for(i=0; i<nEigens; i++)
			{
				float d_i =
					projectedTestFace[i] -
					projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
				//distSq += d_i*d_i / eigenValMat->data.fl[i];  // Mahalanobis
				distSq += d_i*d_i; // Euclidean
			}

			if(distSq < leastDistSq)
			{
				leastDistSq = distSq;
				iNearest = iTrain;
			}
		}

		if (leastDistSq > DIST_THRESH)
			return iNearest;
		else
			return -1;//it's a stranger
	}

	int loadFaceImgArray(char * filename)
	{
		FILE * imgListFile = 0;
		char imgFilename[512];
		int iFace, nFaces=0;

		// open the input file
		if( !(imgListFile = fopen(filename, "r")) )
		{
			fprintf(stderr, "Can\'t open file %s\n", filename);
			return 0;
		}

		// count the number of faces
		while( fgets(imgFilename, 512, imgListFile) ) ++nFaces;
		rewind(imgListFile);
 
		char name[256];

		for(iFace=0; iFace<nFaces; iFace++)
		{
			fscanf(imgListFile,
				"%s %s", &name, imgFilename);

			// load the face image
			IplImage* img = cvLoadImage(imgFilename, CV_LOAD_IMAGE_GRAYSCALE);

			add_profile(img, name);

			if( !img )
			{
				fprintf(stderr, "Can\'t load image from %s\n", imgFilename);
			}
		}

		fclose(imgListFile);

		return nFaces;
	}

};
