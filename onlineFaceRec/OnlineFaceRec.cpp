//////////////////////////////////////////////////////////////////////////////////////
// OnlineFaceRec.cpp, by Shervin Emami (www.shervinemami.co.cc) on 2nd June 2010.
// Online Face Recognition from a camera using Eigenfaces.
//////////////////////////////////////////////////////////////////////////////////////
//
// Some parts are based on the code example by Robin Hewitt (2007) at:
// "http://www.cognotics.com/opencv/servo_2007_series/part_5/index.html"
//
// Command-line Usage (for offline mode, without a webcam):
//
// First, you need some face images. I used the ORL face database.
// You can download it for free at
//    www.cl.cam.ac.uk/research/dtg/attarchive/facedatabase.html
//
// List the training and test face images you want to use in the
// input files train.txt and test.txt. (Example input files are provided
// in the download.) To use these input files exactly as provided, unzip
// the ORL face database, and place train.txt, test.txt, and eigenface.exe
// at the root of the unzipped database.
//
// To run the learning phase of eigenface, enter in the command prompt:
//    OnlineFaceRec train <train_file>
// To run the recognition phase, enter:
//    OnlineFaceRec test <test_file>
// To run online recognition from a camera, enter:
//    OnlineFaceRec
//
//////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <conio.h>		// For _kbhit()
#include <direct.h>		// For mkdir()
#include <vector>
#include <string>
//#include <string.h>
#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/highgui.h"

#if defined _DEBUG
#pragma comment(lib,"opencv_core231d.lib")
#pragma comment(lib,"opencv_imgproc231d.lib")
#pragma comment(lib,"opencv_highgui231d.lib")
#pragma comment(lib,"opencv_features2d231d.lib")
#pragma comment(lib,"opencv_calib3d231d.lib")
#pragma comment(lib,"opencv_objdetect231d.lib")
#pragma comment(lib,"opencv_legacy231d.lib")
#else
#pragma comment(lib,"opencv_core231.lib")
#pragma comment(lib,"opencv_imgproc231.lib")
#pragma comment(lib,"opencv_highgui231.lib")
#pragma comment(lib,"opencv_features2d231.lib")
#pragma comment(lib,"opencv_calib3d231.lib")
#pragma comment(lib,"opencv_objdetect231.lib")
#pragma comment(lib,"opencv_legacy231.lib")
#endif

using namespace std;

// Haar Cascade file, used for Face Detection.
const char *faceCascadeFilename = "haarcascade_frontalface_alt.xml";

int SAVE_EIGENFACE_IMAGES = 1;		// Set to 0 if you dont want images of the Eigenvectors saved to files (for debugging).
//#define USE_MAHALANOBIS_DISTANCE	// You might get better recognition accuracy if you enable this.


// Global variables
IplImage ** faceImgArr        = 0; // array of face images
CvMat    *  personNumTruthMat = 0; // array of person numbers
//#define	MAX_NAME_LENGTH 256		// Give each name a fixed size for easier code.
//char **personNames = 0;			// array of person names (indexed by the person number). Added by Shervin.
vector<string> personNames;			// array of person names (indexed by the person number). Added by Shervin.
int faceWidth = 120;	// Default dimensions for faces in the face recognition database. Added by Shervin.
int faceHeight = 90;	//	"		"		"		"		"		"		"		"
int nPersons                  = 0; // the number of people in the training set. Added by Shervin.
int nTrainFaces               = 0; // the number of training images
int nEigens                   = 0; // the number of eigenvalues
IplImage * pAvgTrainImg       = 0; // the average image
IplImage ** eigenVectArr      = 0; // eigenvectors
CvMat * eigenValMat           = 0; // eigenvalues
CvMat * projectedTrainFaceMat = 0; // projected training faces

CvCapture* camera = 0;	// The camera device.


// Function prototypes
void printUsage();
void learn(char *szFileTrain);
void doPCA();
void storeTrainingData();
int  loadTrainingData(CvMat ** pTrainPersonNumMat);
int  findNearestNeighbor(float * projectedTestFace);
int findNearestNeighbor(float * projectedTestFace, float *pConfidence);
int  loadFaceImgArray(char * filename);
void recognizeFileList(char *szFileTest);
void recognizeFromCam(void);
IplImage* getCameraFrame(void);
IplImage* convertImageToGreyscale(const IplImage *imageSrc);
IplImage* cropImage(const IplImage *img, const CvRect region);
IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight);
IplImage* convertFloatImageToUcharImage(const IplImage *srcImg);
void saveFloatImage(const char *filename, const IplImage *srcImg);
CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade );
CvMat* retrainOnline(void);

// Show how to use this program from the command-line.
void printUsage()
{
	printf("OnlineFaceRec, created by Shervin Emami (www.shervinemami.co.cc), 2nd Jun 2010.\n"
		"Usage: OnlineFaceRec [<command>] \n"
		"  Valid commands are: \n"
		"    train <train_file> \n"
		"    test <test_file> \n"
		" (if no args are supplied, then online camera mode is enabled).\n"
		);
}


// Startup routine.
int main( int argc, char** argv )
{
	printUsage();

	if( argc >= 2 && strcmp(argv[1], "train") == 0 ) {
		char *szFileTrain;
		if (argc == 3)
			szFileTrain = argv[2];	// use the given arg
		else {
			printf("ERROR: No training file given.\n");
			return 1;
		}
		learn(szFileTrain);
	}
	else if( argc >= 2 && strcmp(argv[1], "test") == 0) {
		char *szFileTest;
		if (argc == 3)
			szFileTest = argv[2];	// use the given arg
		else {
			printf("ERROR: No testing file given.\n");
			return 1;
		}
		recognizeFileList(szFileTest);
	}
	else {
		recognizeFromCam();
	}
	return 0;
}

// Save all the eigenvectors as images, so that they can be checked.
void storeEigenfaceImages()
{
	// Store the average image to a file
	printf("Saving the image of the average face as 'out_averageImage.bmp'.\n");
	cvSaveImage("out_averageImage.bmp", pAvgTrainImg);
	// Create a large image made of many eigenface images.
	// Must also convert each eigenface image to a normal 8-bit UCHAR image instead of a 32-bit float image.
	printf("Saving the %d eigenvector images as 'out_eigenfaces.bmp'\n", nEigens);
	if (nEigens > 0) {
		// Put all the eigenfaces next to each other.
		int COLUMNS = 8;	// Put upto 8 images on a row.
		int nCols = min(nEigens, COLUMNS);
		int nRows = 1 + (nEigens / COLUMNS);	// Put the rest on new rows.
		int w = eigenVectArr[0]->width;
		int h = eigenVectArr[0]->height;
		CvSize size;
		size = cvSize(nCols * w, nRows * h);
		IplImage *bigImg = cvCreateImage(size, IPL_DEPTH_8U, 1);	// 8-bit Greyscale UCHAR image
		for (int i=0; i<nEigens; i++) {
			// Get the eigenface image.
			IplImage *byteImg = convertFloatImageToUcharImage(eigenVectArr[i]);
			// Paste it into the correct position.
			int x = w * (i % COLUMNS);
			int y = h * (i / COLUMNS);
			CvRect ROI = cvRect(x, y, w, h);
			cvSetImageROI(bigImg, ROI);
			cvCopyImage(byteImg, bigImg);
			cvResetImageROI(bigImg);
			cvReleaseImage(&byteImg);
		}
		cvSaveImage("out_eigenfaces.bmp", bigImg);
		cvReleaseImage(&bigImg);
	}
}

// Train from the data in the given text file, and store the trained data into the file 'facedata.xml'.
void learn(char *szFileTrain)
{
	int i, offset;

	// load training data
	printf("Loading the training images in '%s'\n", szFileTrain);
	nTrainFaces = loadFaceImgArray(szFileTrain);
	printf("Got %d training images.\n", nTrainFaces);
	if( nTrainFaces < 2 )
	{
		fprintf(stderr,
		        "Need 2 or more training faces\n"
		        "Input file contains only %d\n", nTrainFaces);
		return;
	}

	// do PCA on the training faces
	doPCA();

	// project the training images onto the PCA subspace
	projectedTrainFaceMat = cvCreateMat( nTrainFaces, nEigens, CV_32FC1 );
	offset = projectedTrainFaceMat->step / sizeof(float);
	for(i=0; i<nTrainFaces; i++)
	{
		//int offset = i * nEigens;
		cvEigenDecomposite(
			faceImgArr[i],
			nEigens,
			eigenVectArr,
			0, 0,
			pAvgTrainImg,
			//projectedTrainFaceMat->data.fl + i*nEigens);
			projectedTrainFaceMat->data.fl + i*offset);
	}

	// store the recognition data as an xml file
	storeTrainingData();

	// Save all the eigenvectors as images, so that they can be checked.
	if (SAVE_EIGENFACE_IMAGES) {
		storeEigenfaceImages();
	}

}


// Open the training data from the file 'facedata.xml'.
int loadTrainingData(CvMat ** pTrainPersonNumMat)
{
	CvFileStorage * fileStorage;
	int i;

	// create a file-storage interface
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_READ );
	if( !fileStorage ) {
		printf("Can't open training database file 'facedata.xml'.\n");
		return 0;
	}

	// Load the person names. Added by Shervin.
	personNames.clear();	// Make sure it starts as empty.
	nPersons = cvReadIntByName( fileStorage, 0, "nPersons", 0 );
	if (nPersons == 0) {
		printf("No people found in the training database 'facedata.xml'.\n");
		return 0;
	}
	// Load each person's name.
	for (i=0; i<nPersons; i++) {
		string sPersonName;
		char varname[200];
		sprintf( varname, "personName_%d", (i+1) );
		sPersonName = cvReadStringByName(fileStorage, 0, varname );
		personNames.push_back( sPersonName );
	}

	// Load the data
	nEigens = cvReadIntByName(fileStorage, 0, "nEigens", 0);
	nTrainFaces = cvReadIntByName(fileStorage, 0, "nTrainFaces", 0);
	*pTrainPersonNumMat = (CvMat *)cvReadByName(fileStorage, 0, "trainPersonNumMat", 0);
	eigenValMat  = (CvMat *)cvReadByName(fileStorage, 0, "eigenValMat", 0);
	projectedTrainFaceMat = (CvMat *)cvReadByName(fileStorage, 0, "projectedTrainFaceMat", 0);
	pAvgTrainImg = (IplImage *)cvReadByName(fileStorage, 0, "avgTrainImg", 0);
	eigenVectArr = (IplImage **)cvAlloc(nTrainFaces*sizeof(IplImage *));
	for(i=0; i<nEigens; i++)
	{
		char varname[200];
		sprintf( varname, "eigenVect_%d", i );
		eigenVectArr[i] = (IplImage *)cvReadByName(fileStorage, 0, varname, 0);
	}

	// release the file-storage interface
	cvReleaseFileStorage( &fileStorage );

	printf("Training data loaded (%d training images of %d people):\n", nTrainFaces, nPersons);
	printf("People: ");
	if (nPersons > 0)
		printf("<%s>", personNames[0].c_str());
	for (i=1; i<nPersons; i++) {
		printf(", <%s>", personNames[i].c_str());
	}
	printf(".\n");

	return 1;
}


// Save the training data to the file 'facedata.xml'.
void storeTrainingData()
{
	CvFileStorage * fileStorage;
	int i;

	// create a file-storage interface
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_WRITE );

	// Store the person names. Added by Shervin.
	cvWriteInt( fileStorage, "nPersons", nPersons );
	for (i=0; i<nPersons; i++) {
		char varname[200];
		sprintf( varname, "personName_%d", (i+1) );
		cvWriteString(fileStorage, varname, personNames[i].c_str(), 0);
	}

	// store all the data
	cvWriteInt( fileStorage, "nEigens", nEigens );
	cvWriteInt( fileStorage, "nTrainFaces", nTrainFaces );
	cvWrite(fileStorage, "trainPersonNumMat", personNumTruthMat, cvAttrList(0,0));
	cvWrite(fileStorage, "eigenValMat", eigenValMat, cvAttrList(0,0));
	cvWrite(fileStorage, "projectedTrainFaceMat", projectedTrainFaceMat, cvAttrList(0,0));
	cvWrite(fileStorage, "avgTrainImg", pAvgTrainImg, cvAttrList(0,0));
	for(i=0; i<nEigens; i++)
	{
		char varname[200];
		sprintf( varname, "eigenVect_%d", i );
		cvWrite(fileStorage, varname, eigenVectArr[i], cvAttrList(0,0));
	}

	// release the file-storage interface
	cvReleaseFileStorage( &fileStorage );
}

// Find the most likely person based on a detection. Returns the index, and stores the confidence value into pConfidence.
int findNearestNeighbor(float * projectedTestFace, float *pConfidence)
{
	//double leastDistSq = 1e12;
	double leastDistSq = DBL_MAX;
	int i, iTrain, iNearest = 0;

	for(iTrain=0; iTrain<nTrainFaces; iTrain++)
	{
		double distSq=0;

		for(i=0; i<nEigens; i++)
		{
			float d_i = projectedTestFace[i] - projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
#ifdef USE_MAHALANOBIS_DISTANCE
			distSq += d_i*d_i / eigenValMat->data.fl[i];  // Mahalanobis distance (might give better results than Eucalidean distance)
#else
			distSq += d_i*d_i; // Euclidean distance.
#endif
		}

		if(distSq < leastDistSq)
		{
			leastDistSq = distSq;
			iNearest = iTrain;
		}
	}

	// Return the confidence level based on the Euclidean distance,
	// so that similar images should give a confidence between 0.5 to 1.0,
	// and very different images should give a confidence between 0.0 to 0.5.
	*pConfidence = 1.0f - sqrt( leastDistSq / (float)(nTrainFaces * nEigens) ) / 255.0f;

	// Return the found index.
	return iNearest;
}

// Do the Principal Component Analysis, finding the average image
// and the eigenfaces that represent any image in the given dataset.
void doPCA()
{
	int i;
	CvTermCriteria calcLimit;
	CvSize faceImgSize;

	// set the number of eigenvalues to use
	nEigens = nTrainFaces-1;

	// allocate the eigenvector images
	faceImgSize.width  = faceImgArr[0]->width;
	faceImgSize.height = faceImgArr[0]->height;
	eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);
	for(i=0; i<nEigens; i++)
		eigenVectArr[i] = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

	// allocate the eigenvalue array
	eigenValMat = cvCreateMat( 1, nEigens, CV_32FC1 );

	// allocate the averaged image
	pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

	// set the PCA termination criterion
	calcLimit = cvTermCriteria( CV_TERMCRIT_ITER, nEigens, 1);

	// compute average image, eigenvalues, and eigenvectors
	cvCalcEigenObjects(
		nTrainFaces,
		(void*)faceImgArr,
		(void*)eigenVectArr,
		CV_EIGOBJ_NO_CALLBACK,
		0,
		0,
		&calcLimit,
		pAvgTrainImg,
		eigenValMat->data.fl);

	cvNormalize(eigenValMat, eigenValMat, 1, 0, CV_L1, 0);
}

// Read the names & image filenames of people from a text file, and load all those images listed.
int loadFaceImgArray(char * filename)
{
	FILE * imgListFile = 0;
	char imgFilename[512];
	int iFace, nFaces=0;
	int i;

	// open the input file
	if( !(imgListFile = fopen(filename, "r")) )
	{
		fprintf(stderr, "Can\'t open file %s\n", filename);
		return 0;
	}

	// count the number of faces
	while( fgets(imgFilename, 512, imgListFile) ) ++nFaces;
	rewind(imgListFile);

	// allocate the face-image array and person number matrix
	faceImgArr        = (IplImage **)cvAlloc( nFaces*sizeof(IplImage *) );
	personNumTruthMat = cvCreateMat( 1, nFaces, CV_32SC1 );

	personNames.clear();	// Make sure it starts as empty.
	nPersons = 0;

	// store the face images in an array
	for(iFace=0; iFace<nFaces; iFace++)
	{
		char personName[256];
		string sPersonName;
		int personNumber;

		// read person number (beginning with 1), their name and the image filename.
		fscanf(imgListFile, "%d %s %s", &personNumber, personName, imgFilename);
		sPersonName = personName;
		//printf("Got %d: %d, <%s>, <%s>.\n", iFace, personNumber, personName, imgFilename);

		// Check if a new person is being loaded.
		if (personNumber > nPersons) {
			// Allocate memory for the extra person (or possibly multiple), using this new person's name.
			for (i=nPersons; i < personNumber; i++) {
				personNames.push_back( sPersonName );
			}
			nPersons = personNumber;
			//printf("Got new person <%s> -> nPersons = %d [%d]\n", sPersonName.c_str(), nPersons, personNames.size());
		}

		// Keep the data
		personNumTruthMat->data.i[iFace] = personNumber;

		// load the face image
		faceImgArr[iFace] = cvLoadImage(imgFilename, CV_LOAD_IMAGE_GRAYSCALE);

		if( !faceImgArr[iFace] )
		{
			fprintf(stderr, "Can\'t load image from %s\n", imgFilename);
			return 0;
		}
	}

	fclose(imgListFile);

	printf("Data loaded from '%s': (%d images of %d people).\n", filename, nFaces, nPersons);
	printf("People: ");
	if (nPersons > 0)
		printf("<%s>", personNames[0].c_str());
	for (i=1; i<nPersons; i++) {
		printf(", <%s>", personNames[i].c_str());
	}
	printf(".\n");

	return nFaces;
}


// Recognize the face in each of the test images given, and compare the results with the truth.
void recognizeFileList(char *szFileTest)
{
	int i, nTestFaces  = 0;         // the number of test images
	CvMat * trainPersonNumMat = 0;  // the person numbers during training
	float * projectedTestFace = 0;
	char *answer;
	int nCorrect = 0;
	int nWrong = 0;
	double timeFaceRecognizeStart;
	double tallyFaceRecognizeTime;
	float confidence;

	// load test images and ground truth for person number
	nTestFaces = loadFaceImgArray(szFileTest);
	printf("%d test faces loaded\n", nTestFaces);

	// load the saved training data
	if( !loadTrainingData( &trainPersonNumMat ) ) return;

	// project the test images onto the PCA subspace
	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );
	timeFaceRecognizeStart = (double)cvGetTickCount();	// Record the timing.
	for(i=0; i<nTestFaces; i++)
	{
		int iNearest, nearest, truth;

		// project the test image onto the PCA subspace
		cvEigenDecomposite(
			faceImgArr[i],
			nEigens,
			eigenVectArr,
			0, 0,
			pAvgTrainImg,
			projectedTestFace);

		iNearest = findNearestNeighbor(projectedTestFace, &confidence);
		truth    = personNumTruthMat->data.i[i];
		nearest  = trainPersonNumMat->data.i[iNearest];

		if (nearest == truth) {
			answer = "Correct";
			nCorrect++;
		}
		else {
			answer = "WRONG!";
			nWrong++;
		}
		printf("nearest = %d, Truth = %d (%s). Confidence = %f\n", nearest, truth, answer, confidence);
	}
	tallyFaceRecognizeTime = (double)cvGetTickCount() - timeFaceRecognizeStart;
	if (nCorrect+nWrong > 0) {
		printf("TOTAL ACCURACY: %d%% out of %d tests.\n", nCorrect * 100/(nCorrect+nWrong), (nCorrect+nWrong));
		printf("TOTAL TIME: %.1fms average.\n", tallyFaceRecognizeTime/((double)cvGetTickFrequency() * 1000.0 * (nCorrect+nWrong) ) );
	}

}


// Grab the next camera frame. Waits until the next frame is ready,
// and provides direct access to it, so do NOT modify the returned image or free it!
// Will automatically initialize the camera on the first frame.
IplImage* getCameraFrame(void)
{
	IplImage *frame;

	// If the camera hasn't been initialized, then open it.
	if (!camera) {
		printf("Acessing the camera ...\n");
		camera = cvCaptureFromCAM( 0 );
		if (!camera) {
			printf("ERROR in getCameraFrame(): Couldn't access the camera.\n");
			exit(1);
		}
		// Try to set the camera resolution
		cvSetCaptureProperty( camera, CV_CAP_PROP_FRAME_WIDTH, 320 );
		cvSetCaptureProperty( camera, CV_CAP_PROP_FRAME_HEIGHT, 240 );
		// Wait a little, so that the camera can auto-adjust itself
		Sleep(1000);	// (in milliseconds)
		frame = cvQueryFrame( camera );	// get the first frame, to make sure the camera is initialized.
		if (frame) {
			printf("Got a camera using a resolution of %dx%d.\n", (int)cvGetCaptureProperty( camera, CV_CAP_PROP_FRAME_WIDTH), (int)cvGetCaptureProperty( camera, CV_CAP_PROP_FRAME_HEIGHT) );
		}
	}

	frame = cvQueryFrame( camera );
	if (!frame) {
		fprintf(stderr, "ERROR in recognizeFromCam(): Could not access the camera or video file.\n");
		exit(1);
		//return NULL;
	}
	return frame;
}

// Return a new image that is always greyscale, whether the input image was RGB or Greyscale.
// Remember to free the returned image using cvReleaseImage() when finished.
IplImage* convertImageToGreyscale(const IplImage *imageSrc)
{
	IplImage *imageGrey;
	// Either convert the image to greyscale, or make a copy of the existing greyscale image.
	// This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
	if (imageSrc->nChannels == 3) {
		imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
		cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
	}
	else {
		imageGrey = cvCloneImage(imageSrc);
	}
	return imageGrey;
}

// Creates a new image copy that is of a desired size.
// Remember to free the new image later.
IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight)
{
	IplImage *outImg = 0;
	int origWidth;
	int origHeight;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
		printf("ERROR in resizeImage: Bad desired image size of %dx%d\n.", newWidth, newHeight);
		exit(1);
	}

	// Scale the image to the new dimensions, even if the aspect ratio will be changed.
	outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
	if (newWidth > origImg->width && newHeight > origImg->height) {
		// Make the image larger
		cvResetImageROI((IplImage*)origImg);
		cvResize(origImg, outImg, CV_INTER_LINEAR);	// CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
	}
	else {
		// Make the image smaller
		cvResetImageROI((IplImage*)origImg);
		cvResize(origImg, outImg, CV_INTER_AREA);	// CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
	}

	return outImg;
}

// Returns a new image that is a cropped version of the original image. 
IplImage* cropImage(const IplImage *img, const CvRect region)
{
	IplImage *imageTmp;
	IplImage *imageRGB;
	CvSize size;
	size.height = img->height;
	size.width = img->width;

	if (img->depth != IPL_DEPTH_8U) {
		printf("ERROR in cropImage: Unknown image depth of %d given in cropImage() instead of 8 bits per pixel.\n", img->depth);
		exit(1);
	}

	// First create a new (color or greyscale) IPL Image and copy contents of img into it.
	imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageTmp, NULL);

	// Create a new image of the detected region
	// Set region of interest to that surrounding the face
	cvSetImageROI(imageTmp, region);
	// Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
	size.width = region.width;
	size.height = region.height;
	imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(imageTmp, imageRGB, NULL);	// Copy just the region.

    cvReleaseImage( &imageTmp );
	return imageRGB;		
}

// Get an 8-bit equivalent of the 32-bit Float image.
// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
IplImage* convertFloatImageToUcharImage(const IplImage *srcImg)
{
	IplImage *dstImg = 0;
	if ((srcImg) && (srcImg->width > 0 && srcImg->height > 0)) {

		// Spread the 32bit floating point pixels to fit within 8bit pixel range.
		double minVal, maxVal;
		cvMinMaxLoc(srcImg, &minVal, &maxVal);

		//cout << "FloatImage:(minV=" << minVal << ", maxV=" << maxVal << ")." << endl;

		// Deal with NaN and extreme values, since the DFT seems to give some NaN results.
		if (cvIsNaN(minVal) || minVal < -1e30)
			minVal = -1e30;
		if (cvIsNaN(maxVal) || maxVal > 1e30)
			maxVal = 1e30;
		if (maxVal-minVal == 0.0f)
			maxVal = minVal + 0.001;	// remove potential divide by zero errors.

		// Convert the format
		dstImg = cvCreateImage(cvSize(srcImg->width, srcImg->height), 8, 1);
		cvConvertScale(srcImg, dstImg, 255.0 / (maxVal - minVal), - minVal * 255.0 / (maxVal-minVal));
	}
	return dstImg;
}

// Store a greyscale floating-point CvMat image into a BMP/JPG/GIF/PNG image,
// since cvSaveImage() can only handle 8bit images (not 32bit float images).
void saveFloatImage(const char *filename, const IplImage *srcImg)
{
	//cout << "Saving Float Image '" << filename << "' (" << srcImg->width << "," << srcImg->height << "). " << endl;
	IplImage *byteImg = convertFloatImageToUcharImage(srcImg);
	cvSaveImage(filename, byteImg);
	cvReleaseImage(&byteImg);
}

// Perform face detection on the input image, using the given Haar cascade classifier.
// Returns a rectangle for the detected region in the given image.
CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade )
{
	const CvSize minFeatureSize = cvSize(20, 20);
	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face.
	const float search_scale_factor = 1.1f;
	IplImage *detectImg;
	IplImage *greyImg = 0;
	CvMemStorage* storage;
	CvRect rc;
	double t;
	CvSeq* rects;
	int i;

	storage = cvCreateMemStorage(0);
	cvClearMemStorage( storage );

	// If the image is color, use a greyscale copy of the image.
	detectImg = (IplImage*)inputImg;	// Assume the input image is to be used.
	if (inputImg->nChannels > 1) 
	{
		greyImg = cvCreateImage(cvSize(inputImg->width, inputImg->height), IPL_DEPTH_8U, 1 );
		cvCvtColor( inputImg, greyImg, CV_BGR2GRAY );
		detectImg = greyImg;	// Use the greyscale version as the input.
	}

	// Detect all the faces.
	t = (double)cvGetTickCount();
	rects = cvHaarDetectObjects( detectImg, (CvHaarClassifierCascade*)cascade, storage,
				search_scale_factor, 3, flags, minFeatureSize );
	t = (double)cvGetTickCount() - t;
	printf("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );

	// Get the first detected face (the biggest).
	if (rects->total > 0) {
        rc = *(CvRect*)cvGetSeqElem( rects, 0 );
    }
	else
		rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.

	//cvReleaseHaarClassifierCascade( &cascade );
	//cvReleaseImage( &detectImg );
	if (greyImg)
		cvReleaseImage( &greyImg );
	cvReleaseMemStorage( &storage );

	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
}

// Re-train the new face rec database without shutting down.
// Depending on the number of images in the training set and number of people, it might take 30 seconds or so.
CvMat* retrainOnline(void)
{
	CvMat *trainPersonNumMat;
	int i;

	// Free & Re-initialize the global variables.
	if (faceImgArr) {
		for (i=0; i<nTrainFaces; i++) {
			if (faceImgArr[i])
				cvReleaseImage( &faceImgArr[i] );
		}
	}
	cvFree( &faceImgArr ); // array of face images
	cvFree( &personNumTruthMat ); // array of person numbers
	personNames.clear();			// array of person names (indexed by the person number). Added by Shervin.
	nPersons = 0; // the number of people in the training set. Added by Shervin.
	nTrainFaces = 0; // the number of training images
	nEigens = 0; // the number of eigenvalues
	cvReleaseImage( &pAvgTrainImg ); // the average image
	for (i=0; i<nTrainFaces; i++) {
		if (eigenVectArr[i])
			cvReleaseImage( &eigenVectArr[i] );
	}
	cvFree( &eigenVectArr ); // eigenvectors
	cvFree( &eigenValMat ); // eigenvalues
	cvFree( &projectedTrainFaceMat ); // projected training faces

	// Retrain from the data in the files
	printf("Retraining with the new person ...\n");
	learn("train.txt");
	printf("Done retraining.\n");

	// Load the previously saved training data
	if( !loadTrainingData( &trainPersonNumMat ) ) {
		printf("ERROR in recognizeFromCam(): Couldn't load the training data!\n");
		exit(1);
	}

	return trainPersonNumMat;
}

// Continuously recognize the person in the camera.
void recognizeFromCam(void)
{
	int i;
	CvMat * trainPersonNumMat;  // the person numbers during training
	float * projectedTestFace;
	double timeFaceRecognizeStart;
	double tallyFaceRecognizeTime;
	CvHaarClassifierCascade* faceCascade;
	char cstr[256];
	BOOL saveNextFaces = FALSE;
	char newPersonName[256];
	int newPersonFaces;

	trainPersonNumMat = 0;  // the person numbers during training
	projectedTestFace = 0;
	saveNextFaces = FALSE;
	newPersonFaces = 0;

	printf("Recognizing person in the camera ...\n");

	// Load the previously saved training data
	if( loadTrainingData( &trainPersonNumMat ) ) {
		faceWidth = pAvgTrainImg->width;
		faceHeight = pAvgTrainImg->height;
	}
	else {
		//printf("ERROR in recognizeFromCam(): Couldn't load the training data!\n");
		//exit(1);
	}

	// Project the test images onto the PCA subspace
	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );

	// Create a GUI window for the user to see the camera image.
	cvNamedWindow("Input", CV_WINDOW_AUTOSIZE);

	// Make sure there is a "data" folder, for storing the new person.
	mkdir("data");

	// Load the HaarCascade classifier for face detection.
	faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0 );
	if( !faceCascade ) {
		printf("ERROR in recognizeFromCam(): Could not load Haar cascade Face detection classifier in '%s'.\n", faceCascadeFilename);
		exit(1);
	}

	timeFaceRecognizeStart = (double)cvGetTickCount();	// Record the timing.

	while (1)
	{
		int iNearest, nearest, truth;
		IplImage *camImg;
		IplImage *greyImg;
		IplImage *faceImg;
		IplImage *sizedImg;
		IplImage *equalizedImg;
		IplImage *processedFaceImg;
		CvRect faceRect;
		IplImage *shownImg;
		int keyPressed = 0;
		FILE *trainFile;
		float confidence;

		// Handle keyboard input in the console.
		if (_kbhit())
			keyPressed = getch();

		if (keyPressed == VK_ESCAPE) {	// Check if the user hit the 'Escape' key
			break;	// Stop processing input.
		}
		switch (keyPressed) {
			case 'n':	// Add a new person to the training set.
				// Train from the following images.
				printf("Enter your name: ");
				strcpy(newPersonName, "newPerson");
				gets(newPersonName);
				printf("Collecting all images until you hit 't', to start Training the images as '%s' ...\n", newPersonName);
				newPersonFaces = 0;	// restart training a new person
				saveNextFaces = TRUE;
				break;
			case 't':	// Start training
				saveNextFaces = FALSE;	// stop saving next faces.
				// Store the saved data into the training file.
				printf("Storing the training data for new person '%s'.\n", newPersonName);
				// Append the new person to the end of the training data.
				trainFile = fopen("train.txt", "a");
				for (i=0; i<newPersonFaces; i++) {
					sprintf(cstr, "data/%d_%s%d.pgm", nPersons+1, newPersonName, i+1);
					fprintf(trainFile, "%d %s %s\n", nPersons+1, newPersonName, cstr);
				}
				fclose(trainFile);

				// Now there is one more person in the database, ready for retraining.
				//nPersons++;

				//break;
			//case 'r':

				// Re-initialize the local data.
				projectedTestFace = 0;
				saveNextFaces = FALSE;
				newPersonFaces = 0;

				// Retrain from the new database without shutting down.
				// Depending on the number of images in the training set and number of people, it might take 30 seconds or so.
				cvFree( &trainPersonNumMat );	// Free the previous data before getting new data
				trainPersonNumMat = retrainOnline();
				// Project the test images onto the PCA subspace
				cvFree(&projectedTestFace);	// Free the previous data before getting new data
				projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );

				printf("Recognizing person in the camera ...\n");
				continue;	// Begin with the next frame.
				break;
		}

		// Get the camera frame
		camImg = getCameraFrame();
		if (!camImg) {
			printf("ERROR in recognizeFromCam(): Bad input image!\n");
			exit(1);
		}
		// Make sure the image is greyscale, since the Eigenfaces is only done on greyscale image.
		greyImg = convertImageToGreyscale(camImg);

		// Perform face detection on the input image, using the given Haar cascade classifier.
		faceRect = detectFaceInImage(greyImg, faceCascade );
		// Make sure a valid face was detected.
		if (faceRect.width > 0) {
			faceImg = cropImage(greyImg, faceRect);	// Get the detected face image.
			// Make sure the image is the same dimensions as the training images.
			sizedImg = resizeImage(faceImg, faceWidth, faceHeight);
			// Give the image a standard brightness and contrast, in case it was too dark or low contrast.
			equalizedImg = cvCreateImage(cvGetSize(sizedImg), 8, 1);	// Create an empty greyscale image
			cvEqualizeHist(sizedImg, equalizedImg);
			processedFaceImg = equalizedImg;
			if (!processedFaceImg) {
				printf("ERROR in recognizeFromCam(): Don't have input image!\n");
				exit(1);
			}

			// If the face rec database has been loaded, then try to recognize the person currently detected.
			if (nEigens > 0) {
				// project the test image onto the PCA subspace
				cvEigenDecomposite(
					processedFaceImg,
					nEigens,
					eigenVectArr,
					0, 0,
					pAvgTrainImg,
					projectedTestFace);

				// Check which person it is most likely to be.
				iNearest = findNearestNeighbor(projectedTestFace, &confidence);
				nearest  = trainPersonNumMat->data.i[iNearest];

				printf("Most likely person in camera: '%s' (confidence=%f.\n", personNames[nearest-1].c_str(), confidence);

			}//endif nEigens

			// Possibly save the processed face to the training set.
			if (saveNextFaces) {
// MAYBE GET IT TO ONLY TRAIN SOME IMAGES ?
				// Use a different filename each time.
				sprintf(cstr, "data/%d_%s%d.pgm", nPersons+1, newPersonName, newPersonFaces+1);
				printf("Storing the current face of '%s' into image '%s'.\n", newPersonName, cstr);
				cvSaveImage(cstr, processedFaceImg, NULL);
				newPersonFaces++;
			}

			// Free the resources used for this frame.
			cvReleaseImage( &greyImg );
			cvReleaseImage( &faceImg );
			cvReleaseImage( &sizedImg );
			cvReleaseImage( &equalizedImg );
		}

		// Show the data on the screen.
		shownImg = cvCloneImage(camImg);
		if (faceRect.width > 0) {	// Check if a face was detected.
			// Show the detected face region.
			cvRectangle(shownImg, cvPoint(faceRect.x, faceRect.y), cvPoint(faceRect.x + faceRect.width-1, faceRect.y + faceRect.height-1), CV_RGB(0,255,0), 1, 8, 0);
			if (nEigens > 0) {	// Check if the face recognition database is loaded and a person was recognized.
				// Show the name of the recognized person, overlayed on the image below their face.
				CvFont font;
				cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0,1,CV_AA);
				CvScalar textColor = CV_RGB(0,255,255);	// light blue text
				char text[256];
				sprintf_s(text, sizeof(text)-1, "Name: '%s'", personNames[nearest-1].c_str());
				cvPutText(shownImg, text, cvPoint(faceRect.x, faceRect.y + faceRect.height + 15), &font, textColor);
				sprintf_s(text, sizeof(text)-1, "Confidence: %f", confidence);
				cvPutText(shownImg, text, cvPoint(faceRect.x, faceRect.y + faceRect.height + 30), &font, textColor);
			}
		}

		// Display the image.
		cvShowImage("Input", shownImg);

		// Give some time for OpenCV to draw the GUI and check if the user has pressed something in the GUI window.
		keyPressed = cvWaitKey(10);
		if (keyPressed == VK_ESCAPE) {	// Check if the user hit the 'Escape' key in the GUI window.
			break;	// Stop processing input.
		}

		cvReleaseImage( &shownImg );
	}
	tallyFaceRecognizeTime = (double)cvGetTickCount() - timeFaceRecognizeStart;

	// Free the camera and memory resources used.
	cvReleaseCapture( &camera );
	cvReleaseHaarClassifierCascade( &faceCascade );
}
