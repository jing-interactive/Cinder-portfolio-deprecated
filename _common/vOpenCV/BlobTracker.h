/*
* vBlobTracker.h
* by stefanix
* Thanks to touchlib for the best fit algorithm!
*
* This class tracks blobs between frames.
* Most importantly it assignes persistent id to new blobs, correlates
* them between frames and removes them as blobs dissappear. It also
* compensates for ghost frames in which blobs momentarily dissappear.
*
* Based on the trackning it fires events when blobs come into existence,
* move around, and disappear. The object which receives the callbacks
* can be specified with setListener().
*
*/
#pragma once

#include <map>
#include <vector>

#include "Blob.h"

///////////////////////////////////////////////////////////////////////////////////////////

// This cleans up the foreground segmentation mask derived from calls to cvBackCodeBookDiff
//
// mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
//
// OPTIONAL PARAMETERS:
// poly1_hull0	If set, approximate connected component by (DEFAULT) polygon, or else convex hull (false)
// areaScale 	Area = image (width*height)*areaScale.  If contour area < this, delete that contour (DEFAULT: 0.1)
//
void vFindBlobs(cv::Mat& src, vector<vBlob>& blobs, int minArea = 1, int maxArea = 3072000, bool convexHull=false, bool (*sort_func)(const vBlob& a, const vBlob& b)  = NULL);
void vFindBlobs(IplImage *src, vector<vBlob>& blobs, int minArea = 1, int maxArea = 3072000, bool convexHull=false, bool (*sort_func)(const vBlob& a, const vBlob& b)  = NULL);

void vFindBlobs(IplImage *mask,	int minArea = 1, int maxArea = 3072000, bool convexHull=false);//draw trackedBlobs only

void vFindBlobs(IplImage *src, vector<vBlob>& blobs, vector<vector<vDefect>>& defects, int minArea=1, int maxArea=3072000);


// parameters:
//  silh - input video frame
//  dst - resultant motion picture
//  args - optional parameters
vector<vBlob>  vUpdateMhi( IplImage* silh, IplImage* dst);

class vBlobTracker
{
public:
	vBlobTracker();
	void trackBlobs(const vector<vBlob>& newBlobs);

	std::vector<vTrackedBlob>	trackedBlobs; //tracked blobs
	std::vector<vTrackedBlob>  deadBlobs;

private:
	unsigned int						IDCounter;	  //counter of last blob

protected:
	//blob Events
	void doBlobOn(vTrackedBlob& b );
	void doBlobMoved(vTrackedBlob& b );
	void doBlobOff(vTrackedBlob& b );
};

struct vFingerDetector
{
	vFingerDetector();

	bool findFingers(const vBlob& blob, int k = 10);
	bool findHands(const vBlob& smblob, int k = 200);

	float dlh,max;

	int handspos[2];

	vector<cv::Point2f>		ppico;
	vector<cv::Point2f>		smppico;

	vector<float>				kpointcurv;
	vector<float>				smkpointcurv;

	vector<bool>				bfingerRuns;

	vector<cv::Point2f>		lhand;
	vector<cv::Point2f>		rhand;

//	cv::Vec2f	v1,v2,aux1;

	cv::Vec3f	v1D,vxv;
	cv::Vec3f	v2D;

	 float teta,lhd;
};

struct vHaarFinder
{
	vector<vBlob> blobs;
	float scale;
	//
	bool init(char* cascade_name);
	void find(const cv::Mat& img, int minArea = 1, bool findAllFaces = true);

	vHaarFinder();

protected:

	cv::CascadeClassifier _cascade;
};

struct vOpticalFlowLK
{
	//blocksize must be odd
        vOpticalFlowLK(const cv::Mat& gray, int blocksize = 5);

		void update(const cv::Mat& gray);

		cv::point2df flowAtPoint(int x, int y);
		bool flowInRegion(int x, int y, int w, int h, cv::point2df& vec) ;

        //Used to filter noisey or erroneous vectors
        float minVector;
        float maxVector;

        int width;
        int height;

		cv::Mat vel_x;
        cv::Mat vel_y;
		cv::Mat prev;

		int block_size;
};

struct IBackGround
{
	virtual void init(cv::Mat initial, void* param = NULL) = 0;

	virtual void update(cv::Mat image, int mode = 0) = 0;

	virtual void setIntParam(int idx, int value){}
	virtual cv::Mat& getForeground() = 0;
	virtual cv::Mat& getBackground() = 0;

	virtual ~IBackGround(){}
};

struct IAutoBackGround : IBackGround
{
	CvBGStatModel* bg_model;

	IAutoBackGround()
	{
		bg_model = NULL;
	}

	virtual void init(cv::Mat initial, void* param = NULL) = 0;

	virtual void update(cv::Mat image, int mode = 0);

	cv::Mat& getForeground();

	cv::Mat& getBackground();

	virtual ~IAutoBackGround();
};

struct vBackFGDStat: public IAutoBackGround
{
	void init(cv::Mat initial, void* param = NULL);
};

struct vBackGaussian: public IAutoBackGround
{
	void init(cv::Mat initial, void* param = NULL);
};

struct IStaticBackground : IBackGround
{	
	cv::Mat frame;
	cv::Mat bg;
	cv::Mat fore;

	int threshes[2];

	IStaticBackground();

	virtual ~IStaticBackground();
	virtual void setIntParam(int idx, int value);
	cv::Mat& getForeground();
	cv::Mat& getBackground();
};

#define DETECT_BOTH 0
#define DETECT_DARK 1
#define DETECT_BRIGHT 2

struct vBackGrayDiff: public IStaticBackground
{
	void init(cv::Mat initial, void* param = NULL);
	///mode: 0-> ¼ì²âÃ÷Óë°µ 1->¼ì²âºÚ°µ 2->¼ì²âÃ÷ÁÁ
	void update(cv::Mat image, int mode = DETECT_BOTH);
};

struct vBackColorDiff: public IStaticBackground
{
	int nChannels;
	void init(cv::Mat initial, void* param = NULL);

	///mode: 0-> ¼ì²âÃ÷Óë°µ 1->¼ì²âºÚ°µ 2->¼ì²âÃ÷ÁÁ
	void update(cv::Mat image, int mode = DETECT_BOTH);
};

//ÈýÖ¡²îÖµ·¨
struct vThreeFrameDiff: public IStaticBackground
{
	//TODO: how to deal with gray
	cv::Mat grays[3];
	cv::Mat grayDiff ;

	void init(cv::Mat initial, void* param = NULL);

	void update(cv::Mat image, int mode = 0);

	cv::Mat& getForeground();
	cv::Mat& getBackground();
};