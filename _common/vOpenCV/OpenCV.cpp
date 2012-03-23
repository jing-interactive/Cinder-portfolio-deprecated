#include "OpenCV.h"

#define OPENCV_VERSION CVAUX_STR(CV_MAJOR_VERSION)""CVAUX_STR(CV_MINOR_VERSION)""CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef WIN32
#ifdef _DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION"d.lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#else	//_DEBUG
#pragma comment(lib,"opencv_core"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_imgproc"OPENCV_VERSION".lib")
#pragma comment(lib,"opencv_highgui"OPENCV_VERSION".lib")
#endif	//_DEBUG
#endif	//WIN32

#ifdef KINECT 
#include "../clnui/ofxKinectCLNUI.h"
#endif	//KINECT
#ifdef VIDEOINPUT_LIB 
#include "../videoInput/videoInput.h"
#endif	//WIN32

using namespace cv;

#include <set>

void vRotateImage(IplImage* image, float angle, float centreX, float centreY){
   
   CvPoint2D32f centre;
   CvMat *translate = cvCreateMat(2, 3, CV_32FC1);
   cvSetZero(translate);
   centre.x = centreX;
   centre.y = centreY;
   cv2DRotationMatrix(centre, angle, 1.0, translate);
   cvWarpAffine(image, image, translate, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
   cvReleaseMat(&translate);
}

#define NO_FLIP 1000
/*
flip_param -> flip_mode
0 -> NO_FLIP
1 -> 0	:	horizontal
2 -> 1	:	vertical
3 -> -1	:	both
*/
void vFlip(Mat& src, int flipX, int flipY)
{
	assert (flipX == 0 ||flipX == 1);
	assert (flipY == 0 ||flipY == 1);
	static int mapper[2][2] = {{1,-1},{NO_FLIP,0}};
	int code = mapper[flipY][flipX];

	if (code != NO_FLIP)
		flip(src, src, code);
}

void vFastCopyImageTo( const cv::Mat& src, cv::Mat& dst, const cv::Rect& roi )
{
	assert(src.size() == roi.size());
	Mat sub = dst(roi);

	if (src.channels() == 1 && dst.channels() == 3)
	{
		Mat src_clr(src.rows, src.cols, CV_8UC3);
		vColorFul(src, src_clr);
		src_clr.copyTo(sub);
	}
	else
	{
		src.copyTo(sub);
	}
}

void vCopyImageTo(const cv::Mat& src, cv::Mat& dst, const cv::Rect& roi)
{
	Mat sub = dst(roi);

	if (src.channels() == 1 && dst.channels() == 3)
	{
		Mat src_clr(src.rows, src.cols, CV_8UC3);
		vColorFul(src, src_clr);
		resize(src_clr, sub, sub.size());
	}
	else
	{
		resize(src, sub, sub.size());
	}
}

void vDrawText(cv::Mat& img, int x,int y,char* str, CvScalar clr)
{
	cv::putText(img, str, cvPoint(x,y), FONT_HERSHEY_SIMPLEX, 0.5, clr);
}

CvScalar default_colors[] =
{
	{{169, 176, 155}},
	{{169, 176, 155}}, 
	{{168, 230, 29}},
	{{200, 0,   0}},
	{{79,  84,  33}}, 
	{{84,  33,  42}},
	{{255, 126, 0}},
	{{215,  86, 0}},
	{{33,  79,  84}}, 
	{{33,  33,  84}},
	{{77,  109, 243}}, 
	{{37,   69, 243}},
	{{77,  109, 243}},
	{{69,  33,  84}},
	{{229, 170, 122}}, 
	{{255, 126, 0}},
	{{181, 165, 213}}, 
	{{71, 222,  76}},
	{{245, 228, 156}}, 
	{{77,  109, 243}}
};

const int sizeOfColors = sizeof(default_colors)/sizeof(CvScalar);
CvScalar vDefaultColor(int idx){ return default_colors[idx%sizeOfColors];}

VideoInput::VideoInput()
{
	_fps = 0;
	_frame = NULL;
	_InputType = From_Count;
	device_id = 0;
}

void VideoInput::showSettingsDialog()
{
#if VIDEOINPUT_LIB
	if (VI)
		VI->showSettingsWindow(device_id);
#endif
}

bool VideoInput::init(int cam_idx)
{
	bool opened = false;
	device_id = cam_idx;
	do 
	{
#ifdef VIDEOINPUT_LIB
		//try direct show directly (VideoInput)
		VI = new videoInput();
		VI->setVerbose(false);
		if (VI)
		{
	//		int numDevices = VI->listDevices();
			if (opened = VI->setupDevice(cam_idx))
				break;
		}	
#endif	//WIN32
		_capture.open(CV_CAP_DSHOW+cam_idx); 
		if (_capture.isOpened())
		{
			_InputType = From_Camera;
			sprintf(buffer, "Reading from camera # %d via DirectShow.", cam_idx);
			opened = true;
			break;
		}
		else
		{
			_capture.open(cam_idx); 

			if (_capture.isOpened())
			{		
				_InputType = From_Camera;
				sprintf(buffer, "Reading from camera # %d.", cam_idx);
				opened = true;
				break;
			}
			else
			{
				sprintf(buffer, "Failed to open camera # %d", cam_idx);
				opened = false;
				break;
			}
		}
	} while (0);

	if (opened)
	{
		_post_init();
	}
	printf("\n%s\n",buffer);
	return opened;
}

bool VideoInput::init(const std::string& file_name)
{
	bool loaded = false;
#ifdef KINECT
	if (file_name == "kinect")
	{
		bool b = init_kinect();
		if (b)
		{
			_InputType = From_Kinect;
			loaded = true;
			printf("Reading from kinect.\n");
		}
		else
		{
			printf("Failed to open Kinect.\n"
				"You can download the driver from http://codelaboratories.com/get/nui/\n\n");
			return false;
		}
	}
#endif
	if (!loaded)
	{
		_frame = imread(file_name);

		if (!_frame.empty())
		{
			printf("Reading from image %s.\n", file_name);
			_InputType = From_Image;
		}
		else
		{
			_capture = _capture.open(file_name);
			if(_capture.isOpened())
			{
				printf("Reading from video %s.\n", file_name);
				_InputType = From_Video;
			}
			else
			{
				printf("Could not open file %s.\n", file_name);
				return false;
			}
		}
	}

	_post_init();
	return true;

}

bool VideoInput::init(int argc, char** argv)
{
	_argc = argc;
	_argv = argv;
	if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && ::isdigit(argv[1][0])))
		return init( argc == 2 ? argv[1][0] - '0' : 0 );
	else if( argc == 2 )
		return init( argv[1] );
	return false;
}

void VideoInput::resize( int w, int h )
{
	if (_capture.isOpened())
	{
		_capture.set(CV_CAP_PROP_FRAME_WIDTH, (double)w);
		_capture.set(CV_CAP_PROP_FRAME_HEIGHT, (double)h);
		_post_init();
		return;
	}

#ifdef VIDEOINPUT_LIB
	if( w != VI->getWidth(device_id) || h != VI->getHeight(device_id) )
	{
		VI->stopDevice(device_id);
		VI->setupDevice(device_id, w, h);
		_post_init();
	}
#endif // WIN32
}

void VideoInput::wait(int t)
{
	if (_InputType == From_Image)
		return;
	for (int i=0;i<t;i++)
		get_frame();
}

Mat VideoInput::get_frame()
{
	do 
	{
#ifdef KINECT
		if (_kinect)
		{
			bool b = _kinect->getDepthBW();
			_frame = _kinect->bwImage;
			_frame_num ++;
			break;
		}
#endif
#ifdef VIDEOINPUT_LIB
		if (VI)
		{
			VI->getPixels( device_id, _frame.ptr(), false, true );
			break;;
		}
#endif
		if (_capture.isOpened())
		{
			_capture >> _frame;

			// 			if (_frame == NULL)
			// 			{
			// 				cvReleaseCapture(&_capture);
			// 				init(_argc, _argv);
			// 			}
			break;
		}
	} while (0);

	return _frame;
}

void VideoInput::_post_init()
{
#ifdef VIDEOINPUT_LIB
	if (VI)
	{
		int w = VI->getWidth(device_id), h = VI->getHeight(device_id);
		_frame.create( Size(w,h), CV_8UC3);
	}
#endif
	_frame = get_frame();

	if (_InputType == From_Video)
	{
		_fps = _capture.get(CV_CAP_PROP_FPS);
		_codec = _capture.get(CV_CAP_PROP_FOURCC);
		if (_fps == 0)
			printf("Fps: unknown");
		else
			printf("Fps: %d", _fps);
	}
	else
	{
		_codec = CV_FOURCC('D', 'I', 'V', 'X');
		_fps = 24;
	}

	_size.width = _frame.cols;
	_size.height = _frame.rows;
	_half_size.width  = _size.width/2;
	_half_size.height  = _size.height/2;
	_channel = _frame.channels();
	_frame_num = 0;

	printf("Size: <%d,%d>\n",  _size.width, _size.height);
}

#ifdef KINECT
bool VideoInput::init_kinect()
{
	_kinect = new ofxKinectCLNUI;
	return _kinect->initKinect(640, 480, 0, 0);
}
#endif

void vHighPass(const cv::Mat& src, cv::Mat& dst, int blurLevel/* = 10*/, int noiseLevel/* = 3*/)
{
	if (blurLevel > 0 && noiseLevel > 0)
	{
		// create the unsharp mask using a linear average filter
		cv::blur(src, dst, Size(blurLevel*2+1, blurLevel*2+1));

		dst = src - dst;
//		cvSub(src, dst, dst);

		// filter out the noise using a median filter
		cv::medianBlur(dst, dst, noiseLevel*2+1);
	}
	else
		src.copyTo(dst);
}

void vGetPerspectiveMatrix(CvMat*& warp_matrix, cv::Point2f xsrcQuad[4], cv::Point2f xdstQuad[4])
{
	static CvPoint2D32f srcQuad[4];
	static CvPoint2D32f dstQuad[4];
	for (int i=0;i<4;i++)
	{
		srcQuad[i] = xsrcQuad[i];
		dstQuad[i] = xdstQuad[i];
	}

	if (warp_matrix == NULL)
		warp_matrix = cvCreateMat(3, 3, CV_32FC1);
	cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);
}

void vPerspectiveTransform(const CvArr* src, CvArr* dst, cv::Point xsrcQuad[4], cv::Point xdstQuad[4])
{
	static CvPoint2D32f srcQuad[4];
	static CvPoint2D32f dstQuad[4];
	for (int i=0;i<4;i++)
	{
		srcQuad[i] = xsrcQuad[i];
		dstQuad[i] = xdstQuad[i];
	}

	static CvMat* warp_matrix = cvCreateMat(3, 3, CV_32FC1);
	cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);
	cvWarpPerspective(src, dst, warp_matrix);
}

void vPolyLine(cv::Mat& dst, vector<Point>& pts, CvScalar clr, int thick)
{
	int n = pts.size();
	if (n > 1)
	{
		int k =0;
		for (;k<n-1;k++)
		{
			cv::line(dst, pts[k], pts[k+1], clr, thick);
		}
		cv::line(dst, pts[k], pts[0], clr, thick);
	}
}

bool operator < (const Point& a, const Point& b)
{
	return a.x < b.x && a.y < b.y;
}

void vFillPoly(IplImage* img, const vector<Point>& pt_list, const Scalar& clr/* = Scalar(255,255,255)*/)
{
	const Point* pts = &pt_list[0];
	const int npts = pt_list.size();
	Mat mat(img);
	cv::fillPoly(mat, &pts, &npts, 1, clr);
}

void vLinePoly(IplImage* img, const vector<Point>& pt_list, const Scalar& clr/* = Scalar(255,255,255)*/, int thick/* = 1*/)
{
	const Point* pts = &pt_list[0];
	const int npts = pt_list.size();
	Mat mat(img);
	cv::polylines(mat, &pts, &npts, 1, true, clr, thick);
}

void vLinePoly(IplImage* img, const vector<Point2f>& pt_list, const Scalar& clr/* = Scalar(255,255,255)*/, int thick/* = 1*/)
{
	const int npts = pt_list.size();
	Point* pts = new Point[npts];
	for (int i=0;i<npts;i++)
		pts[i] = pt_list[i];

    Mat mat(img);
	cv::polylines(mat, (const Point**)&pts, &npts, 1, true, clr, thick);

	delete[] pts;
}

bool vTestRectHitRect(const Rect& object1, const Rect& object2)
{
	int left1, left2;
	int right1, right2;
	int top1, top2;
	int bottom1, bottom2;

	left1 = object1.x;
	left2 = object2.x;
	right1 = object1.x + object1.width;
	right2 = object2.x + object2.width;
	top1 = object1.y;
	top2 = object2.y;
	bottom1 = object1.y + object1.height;
	bottom2 = object2.y + object2.height;

	if (bottom1 < top2) return false;
	if (top1 > bottom2) return false;

	if (right1 < left2) return false;
	if (left1 > right2) return false;

	return true;
};
