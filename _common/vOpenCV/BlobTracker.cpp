#include "BlobTracker.h"
#include "point2d.h"
#include <list>

#if defined _DEBUG
#pragma comment(lib,"opencv_video232d.lib")
#pragma comment(lib,"opencv_objdetect232d.lib")
#else
#pragma comment(lib,"opencv_video232.lib")
#pragma comment(lib,"opencv_objdetect232.lib")
#endif

using namespace cv;

//Just some convienience macros
#define CV_CVX_WHITE	CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK	CV_RGB(0x00,0x00,0x00)

bool cmp_blob_area(const vBlob& a, const vBlob& b)
{
	return a.area > b.area;
}

#define CVCONTOUR_APPROX_LEVEL  1   // Approx.threshold - the bigger it is, the simpler is the boundary

void vFindBlobs(IplImage *src, vector<vBlob>& blobs, int minArea, int maxArea, bool convexHull, bool (*sort_func)(const vBlob& a, const vBlob& b))
{
	static MemStorage	contour_mem	= NULL;
	static MemStorage	hull_mem	= NULL;
	CvMoments myMoments;

	if( contour_mem.empty() ) 
		contour_mem = cvCreateMemStorage(100);
	else 
		cvClearMemStorage(contour_mem);

	if( hull_mem.empty() ) 
		hull_mem = cvCreateMemStorage(100);
	else 
		cvClearMemStorage(hull_mem);

	blobs.clear();

	CvSeq* contour_list = 0;
	cvFindContours(src,contour_mem,&contour_list, sizeof(CvContour),
		CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

	for (CvSeq* d = contour_list; d != NULL; d=d->h_next)
	{
		bool isHole = false;
		CvSeq* c = d;
		while (c != NULL)
		{
			double area = fabs(cvContourArea( c ));
			if( area > minArea && area < maxArea)
			{
				int length = cvArcLength(c);

				CvSeq* approx;
				if(convexHull) //Convex Hull of the segmentation
					approx = cvConvexHull2(c,hull_mem,CV_CLOCKWISE,1);
				else //Polygonal approximation of the segmentation
					approx = cvApproxPoly(c,sizeof(CvContour),hull_mem,CV_POLY_APPROX_DP, std::min(length*0.003,2.0));

				area = cvContourArea( approx ); //update area
				cvMoments( approx, &myMoments );

				blobs.push_back(vBlob());

				vBlob& obj = blobs[blobs.size()-1];
				//fill the blob structure
				obj.area	= fabs(area);
				obj.length =  length;
				obj.isHole	= isHole;
				obj.box	= cvBoundingRect(approx);
				obj.rotBox = cvMinAreaRect2(approx);
				obj.angle = (90-obj.rotBox.angle)*GRAD_PI2;//in radians

				if (myMoments.m10 > -DBL_EPSILON && myMoments.m10 < DBL_EPSILON)
				{
					obj.center.x = obj.box.x + obj.box.width/2;
					obj.center.y = obj.box.y + obj.box.height/2;
				}
				else
				{
					obj.center.x = myMoments.m10 / myMoments.m00;
					obj.center.y = myMoments.m01 / myMoments.m00;
				}

				// get the points for the blob
				CvPoint           pt;
				CvSeqReader       reader;
				cvStartReadSeq( approx, &reader, 0 );

				for (int k=0;k<approx->total;k++)
				{
					CV_READ_SEQ_ELEM( pt, reader );
					obj.pts.push_back(pt);
				}
			}//END if( area >= minArea)

			if (isHole)
				c = c->h_next;//one_hole->h_next is another_hole
			else
				c = c->v_next;//one_contour->h_next is one_hole
			isHole = true;
		}//END while (c != NULL)
	}

    if (!sort_func) sort_func = &cmp_blob_area;
	std::sort(blobs.begin(), blobs.end(), sort_func);
}



void vFindBlobs(IplImage *src, int minArea, int maxArea, bool convexHull)
{
	static MemStorage	mem_storage	= NULL;

	//FIND CONTOURS AROUND ONLY BIGGER REGIONS
	if( mem_storage.empty() ) mem_storage = cvCreateMemStorage(0);
	else cvClearMemStorage(mem_storage);

	CvContourScanner scanner = cvStartFindContours(src,mem_storage,sizeof(CvContour),CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

	CvSeq* c;
	while( c = cvFindNextContour( scanner ) )
	{
		double area = fabs(cvContourArea( c ));
		if( area > minArea && area < maxArea)
		{
			CvSeq* contour;
			if(convexHull) //Polygonal approximation of the segmentation
				contour = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
			else //Convex Hull of the segmentation
				contour = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);

			cvDrawContours(src,contour,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to src
		}
	}
	cvEndFindContours( &scanner );
}

void vFindBlobs( IplImage *src, vector<vBlob>& blobs, vector<vector<vDefect>>& defects, int minArea/*=1*/, int maxArea/*=3072000*/)
{
	static MemStorage	contour_mem	= NULL;
	static MemStorage	hull_mem	= NULL;

	CvMoments myMoments;

	if( contour_mem.empty() )
		contour_mem = cvCreateMemStorage(100);
	else cvClearMemStorage(contour_mem);

	if( hull_mem.empty() ) 
		hull_mem = cvCreateMemStorage(100);
	else cvClearMemStorage(hull_mem);

	blobs.clear();
	defects.clear();

	CvSeq* contour_list = 0;
	cvFindContours(src,contour_mem,&contour_list, sizeof(CvContour), CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);

	for (CvSeq* d = contour_list; d != NULL; d=d->h_next)
	{
		bool isHole = false;
		CvSeq* c = d;
		while (c != NULL)
		{
			double area = fabs(cvContourArea( c ));
			if( area > minArea && area < maxArea)
			{
				int length = cvArcLength(c);
				cvMoments( c, &myMoments );

				blobs.push_back(vBlob());

				vBlob& obj = blobs.back();
				//fill the blob structure
				obj.area	= area;
				obj.length =  length;
				obj.isHole	= isHole;
				obj.box	= cvBoundingRect(c);
				obj.rotBox = cvMinAreaRect2(c);
				obj.angle = (90-obj.rotBox.angle)*GRAD_PI2;//in radians

				if (myMoments.m10 > -DBL_EPSILON && myMoments.m10 < DBL_EPSILON)
				{
					obj.center.x = obj.box.x + obj.box.width/2;
					obj.center.y = obj.box.y + obj.box.height/2;
				}
				else
				{
					obj.center.x = myMoments.m10 / myMoments.m00;
					obj.center.y = myMoments.m01 / myMoments.m00;
				}

				// get the points for the blob
				CvPoint           pt;
				CvSeqReader       reader;
				cvStartReadSeq( c, &reader, 0 );

				for (int k=0;k<c->total;k++)
				{
					CV_READ_SEQ_ELEM( pt, reader );
					obj.pts.push_back(pt);
				}

				// defect detection
				CvSeq* seqHull = cvConvexHull2( c, hull_mem, CV_COUNTER_CLOCKWISE, 0 );
				CvSeq* defectsSeq = cvConvexityDefects( c, seqHull, NULL );

				CvConvexityDefect defect;
				cvStartReadSeq( defectsSeq, &reader, 0 );
				int numDefects = defectsSeq->total;

				vector<vDefect> one_defect;
				for(int i=0; i<numDefects; ++i){
					CV_READ_SEQ_ELEM( defect, reader );
					CvPoint& startPt = *(defect.start);
					CvPoint& endPt = *(defect.end);
					CvPoint& depthPt = *(defect.depth_point);
					one_defect.push_back(vDefect( startPt, endPt, depthPt, defect.depth));
				}
				defects.push_back(one_defect);
			}//END if( area >= minArea)

			if (isHole)
				c = c->h_next;//one_hole->h_next is another_hole
			else
				c = c->v_next;//one_contour->h_next is one_hole
			isHole = true;
		}//END while (c != NULL)
	}
}

// various tracking parameters (in seconds)
const double MHI_DURATION = 1;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;

vector<vBlob>  vUpdateMhi( IplImage* silh, IplImage* dst )
{
	// temporary images
	static IplImage *mhi = 0; // MHI
	static IplImage *orient = 0; // orientation
	static IplImage *mask = 0; // valid orientation mask
	static IplImage *segmask = 0; // motion segmentation map
	static CvMemStorage* storage = 0; // temporary storage

	double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
	CvSize size = cvSize(silh->width,silh->height); // get current frame size
	CvSeq* seq;
	CvRect comp_rect;
	double count;
	double angle;
	CvPoint center;
	double magnitude;
	CvScalar color;

	vector<vBlob> result;

	// allocate images at the beginning or
	// reallocate them if the frame size is changed
	if( !mhi || mhi->width != size.width || mhi->height != size.height ) {

		cvReleaseImage( &mhi );
		cvReleaseImage( &orient );
		cvReleaseImage( &segmask );
		cvReleaseImage( &mask );

		mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
		cvZero( mhi ); // clear MHI at the beginning
		orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
		segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
		mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
	}

	//    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // and threshold it
	cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI

	// convert MHI to blue 8u image
	cvCvtScale( mhi, mask, 255./MHI_DURATION,
		(MHI_DURATION - timestamp)*255./MHI_DURATION );
	cvZero( dst );
	cvMerge( mask, 0, 0, 0, dst );

	// calculate motion gradient orientation and valid orientation mask
	cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );

	if( !storage )
		storage = cvCreateMemStorage(0);
	else
		cvClearMemStorage(storage);

	// segment motion: get sequence of motion components
	// segmask is marked motion components map. It is not used further
	seq = cvSegmentMotion( mhi, segmask, storage, timestamp, MAX_TIME_DELTA );

	// iterate through the motion components,
	// One more iteration (i == -1) corresponds to the whole image (global motion)
	for(int i = -1; i < seq->total; i++ ) {

		if( i < 0 ) { // case of the whole image
			comp_rect = cvRect( 0, 0, size.width, size.height );
			color = CV_RGB(255,255,255);
			magnitude = 50;
		}
		else { // i-th motion component
			comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
			if( comp_rect.width + comp_rect.height < 75 ) // reject very tiny components
				continue;
			color = CV_RGB(255,0,0);
			magnitude = 25;
		}

		// select component ROI
		cvSetImageROI( silh, comp_rect );
		cvSetImageROI( mhi, comp_rect );
		cvSetImageROI( orient, comp_rect );
		cvSetImageROI( mask, comp_rect );

		// calculate orientation
		angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);
		angle = 360.0 - angle;  // adjust for images with top-left origin

		count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

		cvResetImageROI( mhi );
		cvResetImageROI( orient );
		cvResetImageROI( mask );
		cvResetImageROI( silh );

		// check for the case of little motion
		if( count < comp_rect.width*comp_rect.height * 0.05 )
			continue;

		// draw a clock with arrow indicating the direction
		center = cvPoint( (comp_rect.x + comp_rect.width/2),
			(comp_rect.y + comp_rect.height/2) );

		cvCircle( dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
		cvLine( dst, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
			cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );

		result.push_back(vBlob(comp_rect, center, angle));
	}

	return result;
}

vFingerDetector::vFingerDetector()
{
	//k is used for fingers and k is used for hand detection
	teta=0.f;
	handspos[0]=0;
	handspos[1]=0;
}

bool vFingerDetector::findFingers (const vBlob& blob, int k/* = 10*/)
{
	ppico.clear();
	kpointcurv.clear();
	bfingerRuns.clear();

	int nPts = blob.pts.size();

	point2df	v1,v2;

	for(int i=k; i<nPts-k; i++)
	{
		//calculating angre between vectors
		v1.set(blob.pts[i].x-blob.pts[i-k].x,	blob.pts[i].y-blob.pts[i-k].y);
		v2.set(blob.pts[i].x-blob.pts[i+k].x,	blob.pts[i].y-blob.pts[i+k].y);

		v1D = Vec3f(blob.pts[i].x-blob.pts[i-k].x,	blob.pts[i].y-blob.pts[i-k].y,	0);
		v2D = Vec3f(blob.pts[i].x-blob.pts[i+k].x,	blob.pts[i].y-blob.pts[i+k].y,	0);

		vxv = v1D.cross(v2D);

		v1.normalize();
		v2.normalize();
		teta=v1.getAngleWith(v2);

		//control conditions
		if(fabs(teta) < 40)
		{	//pik?
			if(vxv[2] > 0)
			{
				bfingerRuns.push_back(true);
				//we put the select poins into ppico vector
				ppico.push_back(blob.pts[i]);
				kpointcurv.push_back(teta);
			}
		}
	}
	if(ppico.size()>0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool vFingerDetector::findHands(const vBlob& smblob, int k)
{
	//smppico.clear();
	//kpointcurv.clear();
	//lhand.clear();
	//rhand.clear();
	//
	//cv::Point hcenter=smobj.center;

	//int nPts = smblob.pts.size();
	//for(int i=k; i<nPts-k; i++)
	//{
	//
	//	v1 = Vec2f(smblob.pts[i].x-smblob.pts[i-k].x,	smblob.pts[i].y-smblob.pts[i-k].y);
	//	v2 = Vec2f(smblob.pts[i].x-smblob.pts[i+k].x,	smblob.pts[i].y-smblob.pts[i+k].y);
	//
	//	v1D = Vec3f(smblob.pts[i].x-smblob.pts[i-k].x,	smblob.pts[i].y-smblob.pts[i-k].y,	0);
	//	v2D = Vec3f(smblob.pts[i].x-smblob.pts[i+k].x,	smblob.pts[i].y-smblob.pts[i+k].y,	0);
	//
	//	vxv = v1D.cross(v2D);
	//
	//	v1.normalize();
	//	v2.normalize();
	//
	//	teta=v1.angle(v2);
	//
	//	if(fabs(teta) < 30)
	//	{	//pik?
	//		if(vxv.z > 0)
	//		{
	//			smppico.push_back(smblob.pts[i]);
	//			kpointcurv.push_back(teta);
	//		}
	//	}
	//}
	//for(int i=0; i<smppico.size();i++)
	//{
	//	if(i==0)
	//	{
	//		lhand.push_back(smppico[i]);
	//	}
	//	else
	//	{
	//		aux1.set(smppico[i].x-smppico[0].x,smppico[i].y-smppico[0].y);
	//		dlh=aux1.length();
	//
	//		//we detect left and right hand and
	//
	//		if(dlh<100)
	//		{
	//			lhand.push_back(smppico[i]);
	//		}
	//		if(dlh>100)
	//		{
	//			rhand.push_back(smppico[i]);
	//		}
	//	}
	//}
	////try to find for each hand the point wich is farder to the center of the Blob
	//if(lhand.size()>0)
	//{
	//	aux1.set(lhand[0].x-hcenter.x,lhand[0].y-hcenter.y);
	//	lhd=aux1.length();
	//	max=lhd;
	//	handspos[0]=0;
	//	for(int i=1; i<lhand.size(); i++)
	//	{
	//		aux1.set(lhand[i].x-hcenter.x,lhand[i].y-hcenter.y);
	//		lhd=aux1.length();
	//		if(lhd>max)
	//		{
	//			max=lhd;
	//			handspos[0]=i;
	//		}
	//	}
	//}
	//if(rhand.size()>0)
	//{
	//	aux1.set(rhand[0].x-hcenter.x,rhand[0].y-hcenter.y);
	//	lhd=aux1.length();
	//	max=lhd;
	//	handspos[1]=0;
	//	for(int i=1; i<rhand.size(); i++)
	//	{
	//		aux1.set(rhand[i].x-hcenter.x,rhand[i].y-hcenter.y);
	//		lhd=aux1.length();
	//		if(lhd>max)
	//		{
	//			max=lhd;
	//			handspos[1]=i;
	//		}
	//	}
	//}
	if(rhand.size()>0 || lhand.size()>0) return true;
	return false;
	//Positions of hands are in (lhand[handspos[0]].x, y+lhand[handspos[0]].y) for left hand and (rhand[handspos[1]].x, y+rhand[handspos[1]].y) for right hand
}



vHaarFinder::vHaarFinder()
{ 
	scale = 1.2;
}
 
bool vHaarFinder::init(char* cascade_name)
{
	return _cascade.load(cascade_name);
}


void vHaarFinder::find(IplImage* img, int minArea, bool findAllFaces)
{
	blobs.clear();

	Ptr<IplImage> gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
	Ptr<IplImage> tiny = cvCreateImage( cvSize( cvRound (img->width/scale),
		cvRound (img->height/scale)), 8, 1 );

	vGrayScale(img, gray);
	cvResize( gray, tiny );
	cvEqualizeHist( tiny, tiny );

	vector<Rect> faces;

	_cascade.detectMultiScale( (IplImage*)tiny, faces,
		1.1, 2, 0
		| findAllFaces ? CV_HAAR_DO_CANNY_PRUNING : CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_DO_CANNY_PRUNING
		//|CV_HAAR_FIND_BIGGEST_OBJECT
		//|CV_HAAR_DO_ROUGH_SEARCH
		|CV_HAAR_SCALE_IMAGE
		,
		Size(30, 30) );

	int n_faces = faces.size();

	for(int i = 0; i < n_faces; i++ )
	{
		Rect& _r = faces[i];
		Rect r = Rect(_r.x*scale, _r.y*scale, _r.width*scale,_r.height*scale);

		float area          = r.width * r.height;
		if (area < minArea)
			continue;

		blobs.push_back( vBlob() );

		float length        = (r.width * 2)+(r.height * 2);
		float centerx       = (r.x) + (r.width / 2.0);
		float centery       = (r.y) + (r.height / 2.0);
		blobs[i].area		= fabs(area);
		blobs[i].isHole		= false;
		blobs[i].length		= length;
		blobs[i].box			= r;
		blobs[i].center.x		= (int) centerx;
		blobs[i].center.y		= (int) centery;
	}

	if (findAllFaces)
		std::sort(blobs.begin(), blobs.end(), cmp_blob_area);
}

vOpticalFlowLK::vOpticalFlowLK(IplImage* gray, int blocksize)
{
	width = gray->width;
	height = gray->height;

	vel_x = cvCreateImage( cvSize( width ,height ), IPL_DEPTH_32F, 1  );
	vel_y = cvCreateImage( cvSize( width ,height ), IPL_DEPTH_32F, 1  );

	cvSetZero(vel_x);
	cvSetZero(vel_y);

	prev = cvCreateImage(cvSize( width ,height ), 8, 1);
	if (gray->nChannels == 1)
		cvCopy(gray, prev);
	else
		vGrayScale(gray, prev);

	block_size = blocksize;

	minVector = 0.1;
	maxVector = 10;
}

void vOpticalFlowLK::update(IplImage* gray)
{
	//cout<<"CALC-ING FLOW"<<endl;
	cvCalcOpticalFlowLK( prev, gray,
		cvSize( block_size, block_size), vel_x, vel_y);
	cvCopy(gray, prev);
}

cv::point2df vOpticalFlowLK::flowAtPoint(int x, int y){
	if(x >= width || x < 0 || y >= height || y < 0){
		return point2df(0.0f,0.0f);
	}
	float fdx = cvGetReal2D( vel_x, y, x );
	float fdy = cvGetReal2D( vel_y, y, x );
	float mag2 = fdx*fdx + fdy*fdy;

	if (mag2 > 0)
	{
		int a=0;
	}
	if(  mag2 > maxVector*maxVector){
		//return a normalized vector of the magnitude size
		//return cv::Point2f(fdx,fdy)/mag2 * maxVector;
		return cv::point2df(fdx,fdy) /mag2* maxVector;
	}
	if( mag2 < minVector*minVector){
		//threhsold to 0
		return cv::point2df(0,0);
	}
	return cv::point2df(fdx, fdy);
}

bool vOpticalFlowLK::flowInRegion(int x, int y, int w, int h, cv::point2df& vec)
{
	float fdx = 0;
	float fdy = 0;
	for(int i = 0; i < w; i++){
		for (int j = 0; j < h; j++) {
			fdx += cvGetReal2D( vel_x, y, x );
			fdy += cvGetReal2D( vel_y, y, x );
		}
	}
	fdx /= w*h;
	fdy /= w*h;
	vec = cv::point2df(fdx, fdy);
	if ((fdx*fdx + fdy*fdy) > minVector*minVector)
		return true;
	else
		return false;
}



vBlobTracker::vBlobTracker()
{
	IDCounter = 0;
}
 
//assigns IDs to each blob in the contourFinder
void vBlobTracker::trackBlobs(const vector<vBlob>& _newBlobs)
{
	leaveBlobs.clear();

	vector<vTrackedBlob> newBlobs;
	int nNewBlobs = _newBlobs.size();
	for (int i=0;i<nNewBlobs;i++)
		newBlobs.push_back(vTrackedBlob(_newBlobs[i]));
// 	//initialize ID's of all trackedBlobs
// 	for(int i=0; i<nNewBlobs; i++)
// 		newBlobs[i].id=-1;

	//go through all tracked trackedBlobs to compute nearest new point
	for(int i=0; i<trackedBlobs.size(); i++)
	{
		/******************************************************************
		* *****************TRACKING FUNCTION TO BE USED*******************
		* Replace 'trackKnn(...)' with any function that will take the
		* current track and find the corresponding track in the newBlobs
		* 'winner' should contain the index of the found blob or '-1' if
		* there was no corresponding blob
		*****************************************************************/
		int winner = trackKnn(newBlobs, trackedBlobs[i], KNN);

		if(winner == -1) //track has died, mark it for deletion
		{
			//SEND BLOB OFF EVENT
			doBlobOff( trackedBlobs[i] );
		}
		else //still alive, have to update
		{
			//if winning new blob was labeled winner by another track\
			//then compare with this track to see which is closer
			if(newBlobs[winner].id != vTrackedBlob::BLOB_UN_NAMED)
			{
				//find the currently assigned blob
				int j; //j will be the index of it
				for(j=0; j<trackedBlobs.size(); j++)
				{
					if(trackedBlobs[j].id==newBlobs[winner].id)
						break;
				}

				if(j==trackedBlobs.size())//got to end without finding it
				{
					newBlobs[winner].id = trackedBlobs[i].id;
					Point2f lastCenter = trackedBlobs[i].center;
					trackedBlobs[i] = newBlobs[winner];
					trackedBlobs[i].velocity.x = trackedBlobs[i].center.x - lastCenter.x;
					trackedBlobs[i].velocity.y = trackedBlobs[i].center.y - lastCenter.y;
				}
				else //found it, compare with current blob
				{
					double x = newBlobs[winner].center.x;
					double y = newBlobs[winner].center.y;
					double xOld = trackedBlobs[j].center.x;
					double yOld = trackedBlobs[j].center.y;
					double xNew = trackedBlobs[i].center.x;
					double yNew = trackedBlobs[i].center.y;
					double distOld = (x-xOld)*(x-xOld)+(y-yOld)*(y-yOld);
					double distNew = (x-xNew)*(x-xNew)+(y-yNew)*(y-yNew);

					//if this track is closer, update the ID of the blob
					//otherwise delete this track.. it's dead
					if(distNew<distOld) //update
					{
						newBlobs[winner].id = trackedBlobs[i].id;

						//TODO--------------------------------------------------------------------------
						//now the old winning blob has lost the win.
						//I should also probably go through all the newBlobs
						//at the end of this loop and if there are ones without
						//any winning matches, check if they are close to this
						//one. Right now I'm not doing that to prevent a
						//recursive mess. It'll just be a new track.

						//SEND BLOB OFF EVENT
						doBlobOff( trackedBlobs[j] );
						//------------------------------------------------------------------------------
					}
					else //delete
					{
						//SEND BLOB OFF EVENT
						doBlobOff( trackedBlobs[i] );
					}
				}
			}
			else //no conflicts, so simply update
			{
				newBlobs[winner].id = trackedBlobs[i].id;
			}
		}
	}

	//--Update All Current Tracks
	//remove every track labeled as dead (ID='-1')
	//find every track that's alive and copy it's data from newBlobs
	for(int i=0; i<trackedBlobs.size(); i++)
	{
		if(trackedBlobs[i].id == vTrackedBlob::BLOB_TO_DELETE) //dead
		{
			//erase track
			trackedBlobs.erase(trackedBlobs.begin()+i,
				trackedBlobs.begin()+i+1);

			i--; //decrement one since we removed an element
		}
		else //living, so update it's data
		{
			for(int j=0; j<nNewBlobs; j++)
			{
				if(trackedBlobs[i].id==newBlobs[j].id)
				{
					//update track
					Point2f lastCenter = trackedBlobs[i].center;
					trackedBlobs[i] = newBlobs[j];
					trackedBlobs[i].velocity.x = trackedBlobs[i].center.x - lastCenter.x;
					trackedBlobs[i].velocity.y = trackedBlobs[i].center.y - lastCenter.y;

					//SEND BLOB MOVED EVENT
					doBlobMoved( trackedBlobs[i] );
				}
			}
		}
	}

	//--Add New Living Tracks
	//now every new blob should be either labeled with a tracked ID or\
	//have ID of -1... if the ID is -1... we need to make a new track
	for(int i=0; i<nNewBlobs; i++)
	{
		if(newBlobs[i].id == vTrackedBlob::BLOB_UN_NAMED)
		{
			//add new track
			newBlobs[i].id=IDCounter++;
			trackedBlobs.push_back(newBlobs[i]);

			//SEND BLOB ON EVENT
			doBlobOn( trackedBlobs[i] );
		}
	}
}


/*************************************************************************
* Finds the blob in 'newBlobs' that is closest to the trackedBlob with index
* 'ind' according to the KNN algorithm and returns the index of the winner
* newBlobs	= list of trackedBlobs detected in the latest frame
* track		= current tracked blob being tested
* k			= number of nearest neighbors to consider\
*			  1,3,or 5 are common numbers..\
*			  must always be an odd number to avoid tying
* thresh	= threshold for optimization
**************************************************************************/

int vBlobTracker::trackKnn(const vector<vTrackedBlob>& newBlobs, vTrackedBlob& track, int k, double thresh)
{
	int winner = -1; //initially label track as '-1'=dead
	if((k%2)==0) k++; //if k is not an odd number, add 1 to it

	//if it exists, square the threshold to use as square distance
	if(thresh>0)
		thresh *= thresh;

	//list of neighbor point index and respective distances
	std::list<std::pair<int,double> > nbors;
	std::list<std::pair<int,double> >::iterator iter;

	//find 'k' closest neighbors of testpoint
	double x, y, xT, yT, dist;
	int nNewBlobs = newBlobs.size();

	for(int i=0; i<nNewBlobs; i++)
	{
		x = newBlobs[i].center.x;
		y = newBlobs[i].center.y;
		xT = track.center.x;
		yT = track.center.y;
		dist = (x-xT)*(x-xT)+(y-yT)*(y-yT);

		if(dist<=thresh)//it's good, apply label if no label yet and return
		{
			winner = i;
			return winner;
		}

		/****************************************************************
		* check if this blob is closer to the point than what we've seen
		*so far and add it to the index/distance list if positive
		****************************************************************/

		//search the list for the first point with a longer distance
		//insert sort by new blobs' distance
		for(iter=nbors.begin(); iter!=nbors.end()
			&& dist>=iter->second; iter++);

			if((iter!=nbors.end())||(nbors.size()<k)) //it's valid, insert it
			{
				nbors.insert(iter, 1, std::pair<int, double>(i, dist));//(newblob's id, newblob's dist to track)
				if(nbors.size()>k)//too many items in list, get rid of farthest neighbor
					nbors.pop_back();
			}
	}

	/********************************************************************
	* we now have k nearest neighbors who cast a vote, and the majority
	* wins. we use each class average distance to the target to break any
	* possible ties.
	*********************************************************************/

	// a mapping from labels (IDs) to count/distance
	// votes save the accumulation
	std::map<int, std::pair<int, double> > votes;

	//remember:
	//iter->first = index of newBlob
	//iter->second = distance of newBlob to current tracked blob
	for(iter=nbors.begin(); iter!=nbors.end(); iter++)
	{
		int newBlobId = iter->first;
		double newBlobDist = iter->second;
		//add up how many counts each neighbor got
		int count = ++(votes[newBlobId].first);
		double total_dist = (votes[newBlobId].second+=newBlobDist);

		/* check for a possible tie and break with distance */
		if(count > votes[winner].first || count == votes[winner].first
			&& total_dist < votes[winner].second)
		{
			winner = iter->first;
		}
	}

	return winner;
}


/************************************
* Delegate to Callbacks
*************************************/

void vBlobTracker::doBlobOn( vTrackedBlob& b ) {
	b.status = statusEnter;
	printf("blob: %d enter+\n" , b.id);
}

void vBlobTracker::doBlobMoved( vTrackedBlob& b ) {
	b.status = statusMove;
	//	printf("blob: %d move\n" , b.id);
}

void vBlobTracker::doBlobOff( vTrackedBlob& b ) {
	b.status = statusLeave;
	leaveBlobs.push_back(b);
	printf("blob: %d leave-\n" , b.id);
	b.id = vTrackedBlob::BLOB_TO_DELETE;
}

CvFGDStatModelParams cvFGDStatModelParams()
{
	CvFGDStatModelParams p;
	p.Lc = CV_BGFG_FGD_LC;			/* Quantized levels per 'color' component. Power of two, typically 32, 64 or 128.				*/
	p.N1c = CV_BGFG_FGD_N1C;			/* Number of color vectors used to model normal background color variation at a given pixel.			*/
	p.N2c = CV_BGFG_FGD_N2C;			/* Number of color vectors retained at given pixel.  Must be > N1c, typically ~ 5/3 of N1c.			*/
	/* Used to allow the first N1c vectors to adapt over time to changing background.				*/

	p.Lcc = CV_BGFG_FGD_LCC;			/* Quantized levels per 'color co-occurrence' component.  Power of two, typically 16, 32 or 64.			*/
	p.N1cc = CV_BGFG_FGD_N1CC;		/* Number of color co-occurrence vectors used to model normal background color variation at a given pixel.	*/
	p.N2cc = CV_BGFG_FGD_N2CC;		/* Number of color co-occurrence vectors retained at given pixel.  Must be > N1cc, typically ~ 5/3 of N1cc.	*/
	/* Used to allow the first N1cc vectors to adapt over time to changing background.				*/

	p.is_obj_without_holes;/* If TRUE we ignore holes within foreground blobs. Defaults to TRUE.						*/
	p.perform_morphing;	/* Number of erode-dilate-erode foreground-blob cleanup iterations.						*/
	/* These erase one-pixel junk blobs and merge almost-touching blobs. Default value is 1.			*/

	p.alpha1 = CV_BGFG_FGD_ALPHA_1;		/* How quickly we forget old background pixel values seen.  Typically set to 0.1  				*/
	p.alpha2 = CV_BGFG_FGD_ALPHA_2;		/* "Controls speed of feature learning". Depends on T. Typical value circa 0.005. 				*/
	p.alpha3 = CV_BGFG_FGD_ALPHA_3;		/* Alternate to alpha2, used (e.g.) for quicker initial convergence. Typical value 0.1.				*/

	p.delta = CV_BGFG_FGD_DELTA;		/* Affects color and color co-occurrence quantization, typically set to 2.					*/
	p.T = CV_BGFG_FGD_T;			/* "A percentage value which determines when new features can be recognized as new background." (Typically 0.9).*/
	p.minArea = CV_BGFG_FGD_MINAREA;		/* Discard foreground blobs whose bounding box is tinyer than this threshold.					*/

	return  p;
}

void vBackFGDStat::init(IplImage* initial, void* param)
{
	CvFGDStatModelParams* p = (CvFGDStatModelParams*)param;
	bg_model = cvCreateFGDStatModel( initial, p );
}

void vBackGaussian::init(IplImage* initial, void* param)
{
	CvGaussBGStatModelParams* p = (CvGaussBGStatModelParams*)param;
	bg_model = cvCreateGaussianBGModel( initial, p );
}

void vBackGrayDiff::setIntParam(int idx, int value)
{
	IBackGround::setIntParam(idx, value);
	if (idx == 1)
		dark_thresh = 255-value;
}

void vBackGrayDiff::init(IplImage* initial, void* param/* = NULL*/){
	cv::Size size = cvGetSize(initial);

	Frame.release();
	Frame = cvCreateImage(size, 8, 1);
	Bg.release();
	Bg = cvCreateImage(size, 8, 1);
	Fore.release();
	Fore = cvCreateImage(size, 8, 1);

	thresh = 50;
	dark_thresh = 200;

	if (initial->nChannels == 1)
		cvCopy(initial, Bg);
	else
		vGrayScale(initial, Bg);
}

void vBackGrayDiff::update(IplImage* image, int mode/* = 0*/){
	if (image->nChannels == 1)
		cvCopy(image, Frame);
	else
		vGrayScale(image, Frame); 
	if (mode == DETECT_BOTH)
	{
		BwImage frame(Frame);
		BwImage bg(Bg);
		BwImage fore(Fore);

		cvZero(Fore);
		for (int y=0;y<image->height;y++)
			for (int x=0;x<image->width;x++)
			{
				int delta = frame[y][x] - bg[y][x];
				if (delta >= thresh || delta <= -dark_thresh)
					fore[y][x] = 255;
			}
	}
	else if (mode == DETECT_DARK)
	{
		cvSub(Bg, Frame, Fore);
		vThresh(Fore, dark_thresh);
	}
	else if (mode == DETECT_BRIGHT)
	{
		cvSub(Frame, Bg, Fore);
		vThresh(Fore, thresh);
	}
}


void vBackColorDiff::init(IplImage* initial, void* param/* = NULL*/){
	cv::Size size = cvGetSize(initial);
	nChannels = initial->nChannels;

	Frame.release();
	Frame = cvCloneImage(initial);
	Bg.release();
	Bg = cvCloneImage(initial);
	Fore.release();
	Fore = cvCreateImage(size, 8, 1);

	thresh = 220;
	dark_thresh = 30;
}

void vBackColorDiff::update(IplImage* image, int mode/* = 0*/){
	//	vGrayScale(image, Frame);
	cvCopy(image, Frame);
	if (mode == DETECT_BOTH)
	{
		if (nChannels == 1)
		{
			BwImage frame(Frame);
			BwImage bg(Bg);
			BwImage fore(Fore);

			cvZero(Fore);
			for (int y=0;y<image->height;y++)
				for (int x=0;x<image->width;x++)
				{
					int delta = frame[y][x] - bg[y][x];
					if (delta >= thresh || delta <= -dark_thresh)
						fore[y][x] = 255;
				}
		}
		else
		{
			RgbImage frame(Frame);
			RgbImage bg(Bg);
			BwImage fore(Fore);

			int min_t = 255-thresh;
			int max_t = 255-dark_thresh;
			cvZero(Fore);
			for (int y=0;y<image->height;y++)
				for (int x=0;x<image->width;x++)
				{
					int r = frame[y][x].r - bg[y][x].r;
					int g = frame[y][x].g - bg[y][x].g;
					int b = frame[y][x].b - bg[y][x].b;
#if 1
					if ((r >= thresh || r <= -dark_thresh)
						&& (g >= thresh || g <= -dark_thresh)
						&& (b >= thresh || b <= -dark_thresh))
#else
					int delta = r*r+g*g+b*b;
					if (delta >= min_t*min_t && delta <= max_t*max_t)
#endif
						fore[y][x] = 255;
				}
		}
	}
	else if (mode == DETECT_DARK)
	{
		cvSub(Bg, Frame, Fore);
		vThresh(Fore, dark_thresh);
	}
	else if (mode == DETECT_BRIGHT)
	{
		cvSub(Frame, Bg, Fore);
		vThresh(Fore, thresh);
	}
}

void vThreeFrameDiff::init(IplImage* initial, void* param/* = NULL*/)
{
	cv::Size size = cvGetSize(initial);

	grayFrameOne.release();
	grayFrameOne = cvCreateImage(size, 8, 1);
	vGrayScale(initial, grayFrameOne);
	grayFrameTwo.release();
	grayFrameTwo = cvCreateImage(size, 8, 1);
	vGrayScale(initial, grayFrameTwo);
	grayFrameThree.release();
	grayFrameThree = cvCreateImage(size, 8, 1);
	vGrayScale(initial, grayFrameThree);
	grayDiff.release();
	grayDiff = cvCreateImage(size, 8, 1);
}

void vThreeFrameDiff::update(IplImage* image, int mode/* = 0*/){
	vGrayScale(image, grayFrameThree);

	BwImage one(grayFrameOne);
	BwImage two(grayFrameTwo);
	BwImage three(grayFrameThree);
	BwImage diff(grayDiff);

	cvZero(grayDiff);
	for (int y=0;y<image->height;y++)
		for (int x=0;x<image->width;x++)
		{
			if (abs(one[y][x] - two[y][x]) > thresh ||
				abs(three[y][x] - two[y][x]) > thresh)
				diff[y][x] = 255;
		}

		show_image(grayFrameOne);
		show_image(grayFrameTwo);
		show_image(grayFrameThree);
		cvCopy(grayFrameTwo, grayFrameOne);
		cvCopy(grayFrameThree, grayFrameTwo);

		//if (mode == DETECT_BOTH)
		//	cvAbsDiff(grayFrame, grayBg, grayDiff);
		//else if (mode == DETECT_DARK)
		//	cvSub(grayBg, grayFrame, grayDiff);
		//else if (mode == DETECT_BRIGHT)
		//	cvSub(grayFrame, grayBg, grayDiff);
		//vThresh(grayDiff, thresh);
}
