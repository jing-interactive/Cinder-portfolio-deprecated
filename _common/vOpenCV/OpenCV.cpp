#include "OpenCV.h"

#if defined _DEBUG
#pragma comment(lib,"opencv_core232d.lib")
#pragma comment(lib,"opencv_imgproc232d.lib")
#pragma comment(lib,"opencv_highgui232d.lib")
#else
#pragma comment(lib,"opencv_core232.lib")
#pragma comment(lib,"opencv_imgproc232.lib")
#pragma comment(lib,"opencv_highgui232.lib")
#endif

#ifdef KINECT 
#include "../clnui/ofxKinectCLNUI.h"
#endif
#ifdef PS3 
#include "../CLEye/ofxCLeye.h"
#endif

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
void vFlip(CvArr* src, int flipX, int flipY)
{
	int flip_param = flipX*2 + flipY;
	int mode = NO_FLIP;//NO FLIP
	switch(flip_param)
	{
	case 1:
		mode = 0;break;
	case 2:
		mode = 1;break;
	case 3:
		mode = -1;break;
	default:
		break;
	}
	if (mode != NO_FLIP)
		cvFlip(src, 0, mode);
}
//============================================================================
static bool IsEdgeIn(int ind1, int ind2,
					 const std::vector<std::vector<int> > &edges)
{
	for (int i = 0; i < edges.size (); i++)
	{
		if ((edges[i][0] == ind1 && edges[i][1] == ind2) ||
			(edges[i][0] == ind2 && edges[i][1] == ind1) )
			return true;
	}
	return false;
}

//============================================================================
static bool IsTriangleNotIn(const std::vector<int>& one_tri,
							const std::vector<std::vector<int> > &tris)
{
	std::set<int> tTriangle;
	std::set<int> sTriangle;

	for (int i = 0; i < tris.size (); i ++)
	{
		tTriangle.clear();
		sTriangle.clear();
		for (int j = 0; j < 3; j++ )
		{
			tTriangle.insert(tris[i][j]);
			sTriangle.insert(one_tri[j]);
		}
		if (tTriangle == sTriangle)    return false;
	}

	return true;
}

void vCopyImageTo(CvArr* tiny_image, IplImage* big_image, const CvRect& region)
{
	CV_Assert(tiny_image && big_image);
	// Set the image ROI to display the current image
	cvSetImageROI(big_image, region);

	IplImage* t = (IplImage*)tiny_image;
	if (t->nChannels == 1 && big_image->nChannels == 3)
	{
		Ptr<IplImage> _tiny = cvCreateImage(cvGetSize(t), 8, 3);
		cvCvtColor(tiny_image, _tiny, CV_GRAY2BGR);
		cvResize(_tiny, big_image);
	}
	else
	{
		// Resize the input image and copy the it to the Single Big Image
		cvResize(tiny_image, big_image);
	}
	// Reset the ROI in order to display the next image
	cvResetImageROI(big_image);
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

char* get_time(bool full_length)
{
	static char str[256];
	time_t timep;
	tm *p;
	time(&timep);
	p = gmtime(&timep);

	if (full_length)
		sprintf(str, "%d-%d-%d__%d_%d_%d",
		1900+p->tm_year, 1+p->tm_mon, p->tm_mday,
		p->tm_hour, p->tm_min, p->tm_sec);
	else
		sprintf(str, "%d_%d_%d",
		p->tm_hour, p->tm_min, p->tm_sec);

	return str;
}

void feature_out(IplImage* img, IplImage* mask, int thresh)
{
	int w = img->width;
	int h = img->height;
	int step = img->widthStep;
	int channels = img->nChannels;
	uchar* data   = (uchar *)img->imageData;
	uchar* mdata   = (uchar *)mask->imageData;

	for(int i=0;i<h;i++)
		for(int j=0;j<w;j++)
			for(int k=0;k<channels;k++)
			{
				if (mdata[i*mask->widthStep+j] < thresh)
					data[i*step+j*channels+k] = 0;
			}

}

VideoInput::VideoInput()
{
	_fps = 0;
	_capture = NULL;
	_frame = NULL;
	_InputType = From_Count;
}

void VideoInput::showSettingsDialog()
{
// 	if (_capture)
// 		cvSetCaptureProperty(_capture,CV_CAP_PROP_SHOW_DIALOG, true);
}

bool VideoInput::init(int cam_idx)
{
	bool ret = true;
	do 
	{
		_capture = cvCaptureFromCAM(CV_CAP_DSHOW+cam_idx);

		if( !_capture )
		{
			_capture = cvCaptureFromCAM(cam_idx);

			if (!_capture)
			{
				sprintf(buffer, "Failed to open camera # %d", cam_idx);
				ret = false;
				break;
			}
			else
			{
				_InputType = From_Camera;
				sprintf(buffer, "Reading from camera # %d.", cam_idx);
				_post_init();
				ret = true;
				break;
			}
		}
		else
		{
			_InputType = From_Camera;
			sprintf(buffer, "Reading from camera # %d via DirectShow.", cam_idx);
			_post_init();
			ret = true;
			break;
		}
	} while (0);

	printf("\n%s\n",buffer);
	return ret;
}

bool VideoInput::init(char* file_name)
{
	bool loaded = false;
#ifdef KINECT
	if (strcmp(file_name, "kinect")==0)
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
#ifdef PS3
	if (strcmp(file_name, "ps3")==0)
	{
		bool b = init_ps3();
		if (b)
		{
			_InputType = From_PS3;
			loaded = true;
			printf("Reading from ps3 camera.\n");
		}
		else
		{
			printf("Failed to open ps3 camera.\n"
				"You can download the driver from http://codelaboratories.com/get/cl-eye-driver/\n\n");
			return false;
		}
	}
#endif

	if (!loaded)
	{
		_frame = cvLoadImage(file_name);

		if (_frame)
		{
			printf("Reading from image %s.\n", file_name);
			_InputType = From_Image;
		}
		else
		{
			_capture = cvCaptureFromAVI(file_name);
			if( _capture )
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
	if (_capture)
	{
		cvSetCaptureProperty(_capture,CV_CAP_PROP_FRAME_WIDTH, (double)w);
		cvSetCaptureProperty(_capture,CV_CAP_PROP_FRAME_HEIGHT, (double)h);
		_post_init();
	}
}

void VideoInput::wait(int t)
{
	if (_InputType == From_Image)
		return;
	for (int i=0;i<t;i++)
		get_frame();
}

IplImage* VideoInput::get_frame()
{
	switch (_InputType)
	{
	case From_Camera:
	case From_Video:
		{
			_frame = cvQueryFrame(_capture);
			_frame_num ++;
// 			if (_frame == NULL)
// 			{
// 				cvReleaseCapture(&_capture);
// 				init(_argc, _argv);
// 			}
		}break;
#ifdef KINECT
	case From_Kinect:
		{
			bool b = _kinect->getDepthBW();
			_frame = _kinect->bwImage;
			_frame_num ++;
		}break;
#endif
#ifdef PS3
	case From_PS3:
		{
			bool b = _ps3_cam->getFrame();
			_frame = _ps3_cam->frame;
			_frame_num ++;
		}break;
#endif
	default:
		break;
	}

	return _frame;
}

void VideoInput::_post_init()
{
	_frame = get_frame();

	if (_InputType == From_Video)
	{
		_fps = cvGetCaptureProperty(_capture, CV_CAP_PROP_FPS);
		_codec = cvGetCaptureProperty(_capture, CV_CAP_PROP_FOURCC);
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

	_size.width = _frame->width;
	_size.height = _frame->height;
	_half_size.width  = _size.width/2;
	_half_size.height  = _size.height/2;
	_channel = _frame->nChannels;
	_frame_num = 0;

	printf("Size: <%d,%d>\n",  _size.width, _size.height);
}

VideoInput::~VideoInput()
{
	if (_capture != NULL)
		cvReleaseCapture( &_capture );
	//	if (_frame != NULL)
	//		cvReleaseImage(&_frame);
}

#ifdef KINECT
bool VideoInput::init_kinect()
{
	_kinect = new ofxKinectCLNUI;
	return _kinect->initKinect(640, 480, 0, 0);
}
#endif

#ifdef PS3
bool VideoInput::init_ps3()
{
	_ps3_cam = new ofxCLeye;
	int n_ps3 = ofxCLeye::listDevices();
	if (n_ps3 > 0)
	{
		return _ps3_cam->init(320, 240, false);
	}
	else
	{
		return false;
	}
}
#endif

void vHighPass(IplImage* src, IplImage* dst, int blurLevel/* = 10*/, int noiseLevel/* = 3*/)
{
	if (blurLevel > 0 && noiseLevel > 0)
	{
		// create the unsharp mask using a linear average filter
		cvSmooth(src, dst, CV_BLUR, blurLevel*2+1);

		cvSub(src, dst, dst);

		// filter out the noise using a median filter
		cvSmooth(dst, dst, CV_MEDIAN, noiseLevel*2+1);
	}
	else
		cvCopy(src, dst);
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

static void draw_edge( IplImage* img, CvSubdiv2DEdge edge, CvScalar color )
{
	CvSubdiv2DPoint* org_pt = cvSubdiv2DEdgeOrg(edge);
	CvSubdiv2DPoint* dst_pt = cvSubdiv2DEdgeDst(edge);

	if( org_pt && dst_pt )
	{
		CvPoint2D32f org = org_pt->pt;
		CvPoint2D32f dst = dst_pt->pt;

		CvPoint iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
		CvPoint idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));

		cvLine( img, iorg, idst, color, 1, CV_AA, 0 );
	}
}


static void draw_facet( CvSubdiv2D * subdiv, IplImage * dst, IplImage * src, CvSubdiv2DEdge edge, bool drawLine )
{
	CvSubdiv2DEdge e = edge;
	int i, count = 0;
	vector<CvPoint> buf;

	// count number of edges in facet
	do
	{
		count++;
		e = cvSubdiv2DGetEdge( e, CV_NEXT_AROUND_LEFT );
	}
	while( e != edge && count < subdiv->quad_edges * 4 );

	// gather points
	e = edge;
	assert(count == 3);
	for( i = 0; i < count; i++ )
	{
		CvSubdiv2DPoint *pt = cvSubdiv2DEdgeOrg( e );

		if( !pt )
			break;
		assert( fabs( pt->pt.x ) < 10000 && fabs( pt->pt.y ) < 10000 );
		buf.push_back(cvPoint( cvRound( pt->pt.x ), cvRound( pt->pt.y )));

		e = cvSubdiv2DGetEdge( e, CV_NEXT_AROUND_LEFT );
	}

	if( i == count )
	{
		CvSubdiv2DPoint *pt = cvSubdiv2DEdgeDst( cvSubdiv2DRotateEdge( edge, 1 ));
		if (!pt)
			pt = cvSubdiv2DEdgeOrg( cvSubdiv2DRotateEdge( edge, 0 ));
		CvPoint ip = cvPoint( cvRound( pt->pt.x ), cvRound( pt->pt.y ));
		CvScalar color = {{0,0,0,0}};

		//printf("count = %d, (%d,%d)\n", ip.x, ip.y );

		if( 0 <= ip.x && ip.x < src->width && 0 <= ip.y && ip.y < src->height )
		{
			uchar *ptr = (uchar*)(src->imageData + ip.y * src->widthStep + ip.x * 3);
			color = CV_RGB( ptr[2], ptr[1], ptr[0] );
		}

		cvFillConvexPoly( dst, &buf[0], count, color );

		if (drawLine)
		{
			for (i = 1;i<count;i++)
			{
				cvDrawLine(dst, buf[i], buf[i-1], CV_RGB(30,30,30),1);
			}
		}
	}
}




void vDrawDelaunay( CvSubdiv2D* subdiv,IplImage* src,IplImage * dst , bool drawLine)
{
	int i, total = subdiv->edges->total;

	cvCalcSubdivVoronoi2D( subdiv );

	for( i = 0; i < total; i++ )
	{
		CvQuadEdge2D *edge = (CvQuadEdge2D *) cvGetSetElem( subdiv->edges, i );

		if( edge && CV_IS_SET_ELEM( edge ))
		{
			CvSubdiv2DEdge e = (CvSubdiv2DEdge) edge;

			//	draw_edge( src, (CvSubdiv2DEdge)edge + 1, CV_RGB(0,0,0) );//voroni edge
			//	draw_edge( src, (CvSubdiv2DEdge)edge, CV_RGB(0,0,0) );//delaunay edge

			//e itslef
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 0 ), drawLine);
			//reversed e
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 2 ), drawLine);
		}
	}
}

void vDrawVoroni( CvSubdiv2D * subdiv, IplImage * src, IplImage * dst, bool drawLine )
{
	int i, total = subdiv->edges->total;

	cvCalcSubdivVoronoi2D( subdiv );

	//icvSet( dst, 255 );
	for( i = 0; i < total; i++ )
	{
		CvQuadEdge2D *edge = (CvQuadEdge2D *) cvGetSetElem( subdiv->edges, i );

		if( edge && CV_IS_SET_ELEM( edge ))
		{
			CvSubdiv2DEdge e = (CvSubdiv2DEdge) edge;

			// left
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 1 ), drawLine);
			// right
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 3 ), drawLine);
		}
	}
}



DelaunaySubdiv::DelaunaySubdiv(int w, int h)
{
	storage = cvCreateMemStorage();
	rect = cvRect(0, 0, w, h);

	subdiv = cvCreateSubdivDelaunay2D(rect, storage);
}

void DelaunaySubdiv::insert(float x, float y)
{
	Point2f pt(x,y);
	cvSubdivDelaunay2DInsert(subdiv, pt);
	pt_map.insert(std::make_pair(point2di(x, y), points.size() ) );
	points.push_back(Point(x,y));
}

void DelaunaySubdiv::clear()
{
	cvClearMemStorage(storage);
	subdiv = cvCreateSubdivDelaunay2D(rect, storage);
	pt_map.clear();
	points.clear();
}

void DelaunaySubdiv::intoEdge(CvSubdiv2DEdge edge)
{
	CvSubdiv2DEdge e = edge;
	int i;
	const int count = 3;
	Point triple[count];

	//// count number of edges in facet
	//do
	//{
	//	count++;
	//	e = cvSubdiv2DGetEdge( e, CV_NEXT_AROUND_LEFT );
	//}
	//while( e != edge && count < subdiv->quad_edges * 4 );

	//// gather points
	//e = edge;
	//assert(count == 3);
	for( i = 0; i < count; i++ )
	{
		CvSubdiv2DPoint *pt = cvSubdiv2DEdgeOrg( e );
		if( !pt || fabs( pt->pt.x ) > 1500 || fabs( pt->pt.y ) > 1500)
			break;

		triple[i] = Point(pt->pt.x ,  pt->pt.y);
		e = cvSubdiv2DGetEdge( e, CV_NEXT_AROUND_LEFT );
	}

	if( i == count )
	{
		Triangle aTri;

		for (int k=0;k<3;k++)
		{
			aTri[k] = getIndex(triple[k].x, triple[k].y);
			if (aTri[k] == -1) return;
		}
		aTri.center.X = (triple[0].x + triple[1].x + triple[2].x)/3;
		aTri.center.Y = (triple[0].y + triple[1].y + triple[2].y)/3;

		triangles.push_back(aTri);
	}
}

int  DelaunaySubdiv::getIndex(float x, float y)
{
	int ret = -1;
	map<point2di, int>::const_iterator it =  pt_map.find(point2di(x,y));
	if (it != pt_map.end())
		ret = it->second;
	return ret;
}

void DelaunaySubdiv::buildTriangles()
{
	triangles.clear();
	int i, total = subdiv->edges->total;

	for( i = 0; i < total; i++ )
	{
		CvQuadEdge2D *edge = (CvQuadEdge2D *) cvGetSetElem( subdiv->edges, i );

		if( edge && CV_IS_SET_ELEM( edge ))
		{
			CvSubdiv2DEdge e = (CvSubdiv2DEdge) edge;
			//e itslef
			intoEdge( cvSubdiv2DRotateEdge( e, 0 ));
			//reversed e
			intoEdge( cvSubdiv2DRotateEdge( e, 2 ));
		}
	}
	sort(triangles.begin(),triangles.end());
	std::vector<Triangle>::iterator it = unique(triangles.begin(),triangles.end());
	triangles.erase(it, triangles.end());
}


void DelaunaySubdiv::build()
{
	//	cvCalcSubdivVoronoi2D( subdiv );

	buildTriangles();

	return;

	cv_try_begin();

	int count = points.size();
	int __n = count;
	//hull.resize(count);
	//	hull = Mat(1, count, CV_32FC2);

	//CvPoint* _points =new CvPoint[count];
	//int* _hull = new int[count];
	cv::Mat pointMat( 1, count, CV_32SC2, &points[0] );
	//	 CvMat hullMat = cvMat( 1, count, CV_32SC1, &hull[0]);
	//  for(int  i = 0; i < count; i++ )
	//  {
	//      pt0.x = rand() % (width/2) + width/4;
	//      pt0.y = rand() % (height/2) + height/4;
	//     _points[i] = pt0;
	//points[i] = points[i];
	//  }

	//  cvConvexHull2( &pointMat, ConvexHull, CV_CLOCKWISE, 0 );
	cv::convexHull(pointMat, hull);

	CvMat ConvexHull = cvMat (1, __n, CV_32SC2, &hull[0]);

	//	cv::Mat pointMat(1, count, CV_32SC2, &points[0] );

	//		cv::convexHull(pointMat, hullMat);
	//	convexHull(pointMat, hull);
	//
	////	CvMat* pointMa = cvCreateMat(1, count, CV_32SC2);
	//	cvConvexHull2(&(CvMat)pointMat, ConvexHull, CV_CLOCKWISE, 0);
	//	cv::Mat hullMat(1, hull.size(), CV_32SC2, &hull[0] );
	//
	//	doDelaunay(subdiv,  &ConvexHull);
	//	cvReleaseMat(&pointMa);

	cv_try_end();
}


void DelaunaySubdiv::drawDelaunay( IplImage* src,IplImage * dst , bool drawLine)
{
	int i, total = subdiv->edges->total;

	cvCalcSubdivVoronoi2D( subdiv );

	for( i = 0; i < total; i++ )
	{
		CvQuadEdge2D *edge = (CvQuadEdge2D *) cvGetSetElem( subdiv->edges, i );

		if( edge && CV_IS_SET_ELEM( edge ))
		{
			CvSubdiv2DEdge e = (CvSubdiv2DEdge) edge;

			//	draw_edge( src, (CvSubdiv2DEdge)edge + 1, CV_RGB(0,0,0) );//voroni edge
			//	draw_edge( src, (CvSubdiv2DEdge)edge, CV_RGB(0,0,0) );//delaunay edge

			//e itslef
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 0 ), drawLine);
			//reversed e
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 2 ), drawLine);
		}
	}
}


void DelaunaySubdiv::drawVoroni( IplImage * src, IplImage * dst, bool drawLine )
{
	int i, total = subdiv->edges->total;

	cvCalcSubdivVoronoi2D( subdiv );

	//icvSet( dst, 255 );
	for( i = 0; i < total; i++ )
	{
		CvQuadEdge2D *edge = (CvQuadEdge2D *) cvGetSetElem( subdiv->edges, i );

		if( edge && CV_IS_SET_ELEM( edge ))
		{
			CvSubdiv2DEdge e = (CvSubdiv2DEdge) edge;

			// left
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 1 ), drawLine);
			// right
			draw_facet( subdiv, dst, src, cvSubdiv2DRotateEdge( e, 3 ), drawLine);
		}
	}
}


void on_default(int )
{

}


int BrightnessAdjust(const IplImage* srcImg,
					 IplImage* dstImg,
					 float brightness)
{
	assert(srcImg != NULL);
	assert(dstImg != NULL);

	int h = srcImg->height;
	int w = srcImg->width;
	int n = srcImg->nChannels;

	int x,y,i;
	float val;
	for (i = 0; i < n; i++)//²ÊÉ«Í¼ÏñÐèÒª´¦Àí3¸öÍ¨µÀ£¬»Ò¶ÈÍ¼ÏñÕâÀï¿ÉÒÔÉ¾µô
	{
		for (y = 0; y < h; y++)
		{
			for (x = 0; x < w; x++)
			{

				val = ((uchar*)(srcImg->imageData + srcImg->widthStep*y))[x*n+i];
				val += brightness;
				//¶Ô»Ò¶ÈÖµµÄ¿ÉÄÜÒç³ö½øÐÐ´¦Àí
				if(val>255)	val=255;
				if(val<0) val=0;
				((uchar*)(dstImg->imageData + dstImg->widthStep*y))[x*n+i] = (uchar)val;
			}
		}
	}

	return 0;
}

int ContrastAdjust(const IplImage* srcImg,
				   IplImage* dstImg,
				   float nPercent)
{
	assert(srcImg != NULL);
	assert(dstImg != NULL);

	int h = srcImg->height;
	int w = srcImg->width;
	int n = srcImg->nChannels;

	int x,y,i;
	float val;
	for (i = 0; i < n; i++)//²ÊÉ«Í¼ÏñÐèÒª´¦Àí3¸öÍ¨µÀ£¬»Ò¶ÈÍ¼ÏñÕâÀï¿ÉÒÔÉ¾µô
	{
		for (y = 0; y < h; y++)
		{
			for (x = 0; x < w; x++)
			{

				val = ((uchar*)(srcImg->imageData + srcImg->widthStep*y))[x*n+i];
				val = 128 + (val - 128) * nPercent;
				//¶Ô»Ò¶ÈÖµµÄ¿ÉÄÜÒç³ö½øÐÐ´¦Àí
				if(val>255) val=255;
				if(val<0) val=0;
				((uchar*)(dstImg->imageData + dstImg->widthStep*y))[x*n+i] = (uchar)val;
			}
		}
	}
	return 0;
}

void cvSkinSegment(IplImage* img, IplImage* mask)
{
	CvSize imageSize = cvSize(img->width, img->height);
	IplImage *imgY = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
	IplImage *imgCr = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);
	IplImage *imgCb = cvCreateImage(imageSize, IPL_DEPTH_8U, 1);


	IplImage *imgYCrCb = cvCreateImage(imageSize, img->depth, img->nChannels);
	cvCvtColor(img,imgYCrCb,CV_BGR2YCrCb);
	cvSplit(imgYCrCb, imgY, imgCr, imgCb, 0);
	int y, cr, cb, l, x1, y1, value;
	unsigned char *pY, *pCr, *pCb, *pMask;

	pY = (unsigned char *)imgY->imageData;
	pCr = (unsigned char *)imgCr->imageData;
	pCb = (unsigned char *)imgCb->imageData;
	pMask = (unsigned char *)mask->imageData;
	cvSetZero(mask);
	l = img->height * img->width;
	for (int i = 0; i < l; i++){
		y  = *pY;
		cr = *pCr;
		cb = *pCb;
		cb -= 109;
		cr -= 152
			;
		x1 = (819*cr-614*cb)/32 + 51;
		y1 = (819*cr+614*cb)/32 + 77;
		x1 = x1*41/1024;
		y1 = y1*73/1024;
		value = x1*x1+y1*y1;
		if(y<100)    (*pMask)=(value<700) ? 255:0;
		else        (*pMask)=(value<850)? 255:0;
		pY++;
		pCr++;
		pCb++;
		pMask++;
	}
	cvReleaseImage(&imgY);
	cvReleaseImage(&imgCr);
	cvReleaseImage(&imgCb);
	cvReleaseImage(&imgYCrCb);
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
