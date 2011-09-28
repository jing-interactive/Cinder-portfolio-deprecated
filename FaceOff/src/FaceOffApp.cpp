#include "cinder/app/AppBasic.h"
#include "cinder/Utilities.h"
#include "cinder/Thread.h"
#include "cinder/Capture.h"
#include "cinder/TriMesh.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"

#include "CinderOpenCV.h"
#include "../../_common/vOpenCV/OpenCV.h"
#include "../../_common/vOpenCV/BlobTracker.h"
#include "../../_common/asmlibrary/asmfitting.h"
#include "../../_common/SimpleMutex.h"
#include "../../_common/ciTri/include/ciTri.h"

using namespace ci;
using namespace ci::app;
using namespace std;

bool cmp_rect_area(const Rectf& a, const Rectf& b)
{
	return a.calcArea() > b.calcArea();
}


struct dirty_misc
{
	dirty_misc(){dirty=false;}
	bool dirty;
	void set(){dirty = true;}
	bool get()
	{
		bool ret = dirty;
		if (dirty)
			dirty = false;
		return ret;
	}
};

struct face_element_t
{
	ci::Vec2f* center;
	vector<ci::Vec2f*> items;
	int n_items;
	void init(vector<ci::Vec2f>& points, int idx_center, int* idx_array, int array_size)
	{
		center =&points[idx_center];
		items.resize(array_size);
		for (int i=0;i<array_size;i++)
			items[i] = &points[idx_array[i]];
		n_items = array_size;
	}
	void scale(const ci::Vec2f& k)
	{
		for (int i=0;i<n_items;i++)
		{
			ci::Vec2f dir = *items[i] - *center;
			*items[i] = *center + dir*k;
		}
	}
	void rotate(const ci::Vec2f& k)
	{
	}
	void translate(const ci::Vec2f& k)
	{
	}
}left_eye, right_eye, mouth;

class FaceOffApp : public AppBasic {
public:
	//cinder routines
	void setup();
	void shutdown();
	void mouseDown( MouseEvent event );	
	void keyDown( KeyEvent event );
	void update();
	void draw();

	//input
	void grabCamera();
	std::thread grabCameraThread;
	Capture		mCap;
	ci::Vec2i	camSize;
	Surface		surInput;
	CSimpleMutex mtx_surInput;

	//process
	void detectFace();
	std::thread detectFaceThread;
	void modifyVertices(std::vector<ci::Vec2f>& vertices);
	cv::CascadeClassifier	mFaceCascade;
	vector<Rectf>			mFaces;
	CSimpleEvent			evt_face;
	CSimpleMutex			mtx_mFaces;
	dirty_misc	dirty_face;

	//ASM
	CSimpleEvent evt_asm;
	void runASM();
	std::thread runASMThread;
	asmfitting fit_asm;
	std::vector<TriangleData>	face_indices;
	std::vector<ci::Vec2f>		face_vertices;
	std::vector<ci::Vec2f>		face_vertices2;
	CSimpleMutex				mtx_triList;

	//rendering
	gl::Texture		tex_frame;
	bool running;
	ci::TriMesh triMesh;
	gl::VboMesh vboMesh;
	bool	faceFrameVisible;
};

void FaceOffApp::setup()
{
	running = true;
	faceFrameVisible = false;
	try {
		mCap = Capture( 640, 480 );
		mCap.start();
		camSize = mCap.getSize();
	}
	catch( ... ) {
		console() << "Failed to initialize capture" << std::endl;
		running = false;
		return;
	}

	grabCameraThread = std::thread(&FaceOffApp::grabCamera, this);
	detectFaceThread = std::thread(&FaceOffApp::detectFace, this);
	runASMThread = std::thread(&FaceOffApp::runASM, this);
}

void FaceOffApp::mouseDown( MouseEvent event )
{
}

void FaceOffApp::update()
{
	LOCK_START(mtx_surInput);
	if (surInput)
		tex_frame = surInput;
	LOCK_END();

	LOCK_START(mtx_triList); 
	if (face_vertices.size() > 0)
	{
		triMesh.clear();
		for (int i=0;i<face_vertices.size();i++)
		{
			triMesh.appendVertex(ci::Vec3f(face_vertices[i],0));
		//	triMesh.appendColorRGBA( ColorA( 1, 1, 1,1.0 ) );
			triMesh.appendTexCoord(ci::Vec2f(face_vertices2[i].x/(float)camSize.x,face_vertices2[i].y/(float)camSize.y));
		}
		for (int i=0;i<face_indices.size();i++)
		{
			const TriangleData& tri = face_indices[i];
			triMesh.appendTriangle(tri.idx[0], tri.idx[1], tri.idx[2]);
		}
	}
	LOCK_END();
}

void FaceOffApp::draw()
{
	if (!tex_frame)
		return;

	gl::setMatricesWindow( getWindowSize() );
	gl::enableAlphaBlending();

	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );

	gl::color( ColorA( 1, 1, 1,0.5) );
	gl::draw(tex_frame);

	// draw the faces as transparent yellow circles
	gl::color( ColorA( 1, 1, 0, 0.25f ) );
	LOCK_START(mtx_mFaces);
	for (int i=1;i<mFaces.size();i++) //<--- i begins with 1
	{
		const Rectf& r = mFaces[i];
		gl::drawSolidCircle( r.getCenter(),  (r.getWidth()+r.getHeight())/4);
	}
	LOCK_END();

	if (triMesh.getNumVertices() > 0)//test if..
	{
		gl::color( ColorA( 1, 1, 1,0.5) );
		tex_frame.enableAndBind();
		gl::draw(triMesh);
		tex_frame.unbind();

		if (faceFrameVisible)
		{
			gl::color( Color( 0, 1, 0) );
			gl::enableWireframe();
			gl::draw(triMesh);
			gl::disableWireframe();
		}
	}
}

void FaceOffApp::shutdown()
{
	running = false;

	grabCameraThread.join();
	detectFaceThread.join();
	runASMThread.join();
}


void FaceOffApp::grabCamera()
{
	while (running)
	{
		if( mCap && mCap.checkNewFrame() )
		{
			LOCK_START(mtx_surInput);
			surInput = mCap.getSurface();
			LOCK_END();
			evt_face.set();
		}
		grabCameraThread.yield();
	}
}

void FaceOffApp::detectFace()
{
	if (mFaceCascade.load(getAppPath() + "../../resources/haarcascade_frontalface_alt2.xml"))
		console() << "Opened haar xml" <<std::endl;	
	else
		return;

	vector<cv::Rect> faces;
	cv::Mat grayCameraImage;
	cv::Mat smallImg;
	while (running)
	{
		evt_face.wait();
		const int calcScale = 2; // calculate the image at half scale

		// create a grayscale copy of the input image
		grayCameraImage = toOcv( surInput, CV_8UC1 );

		// scale it to half size, as dictated by the calcScale constant
		int scaledWidth = surInput.getWidth() / calcScale;
		int scaledHeight = surInput.getHeight() / calcScale; 
		if (smallImg.empty())
			smallImg.create( scaledHeight, scaledWidth, CV_8UC1 );
		cv::resize( grayCameraImage, smallImg, smallImg.size());

		// equalize the histogram
		cv::equalizeHist( smallImg, smallImg );

		// detect the faces and iterate them, appending them to mFaces
		mFaceCascade.detectMultiScale( smallImg, faces );
		// clear out the previously detected faces & eyes
		LOCK_START(mtx_mFaces);
		mFaces.clear();
		for( vector<cv::Rect>::const_iterator faceIter = faces.begin(); faceIter != faces.end(); ++faceIter ) {
			Rectf faceRect( fromOcv( *faceIter ) );
			faceRect *= calcScale;
			mFaces.push_back( faceRect );
		}
		std::sort(mFaces.begin(), mFaces.end(), cmp_rect_area);
		LOCK_END();
		//dirty_face.set();
		if (faces.size() > 0)//only after faces detected 
			evt_asm.set();
	}
}

void FaceOffApp::runASM()
{
	std::string model = getAppPath() + "../../resources/my68-1d.amf";
	if (!fit_asm.Read(model.c_str()))
		return;
	std::vector<asm_shape> shape_list;//fitting shapes
	std::vector<asm_shape> detected_list;//detected shapes
	int n_shapes = 0;
	const int n_iterations = 20;
	int fr_asm_frame = 0;
	cv::Mat frame;//a copy of camera input
	std::vector<ci::Vec2f> facePoints;

	while (running)
	{
		evt_asm.wait();
		fr_asm_frame++;
		frame = toOcv( surInput, CV_8UC3 );

		LOCK_START(mtx_mFaces); 
		//n_shapes = mFaces.size();
		n_shapes = 1;
		detected_list.resize(n_shapes);
		for (int i = 0; i < n_shapes; i++)
		{
			const Rectf& r = mFaces[i];
			detected_list[i].Resize(2); 
			detected_list[i][0].x  = r.x1;
			detected_list[i][0].y  = r.y1;
			detected_list[i][1].x  = r.x2;
			detected_list[i][1].y  = r.y2;
		}
		LOCK_END();

		shape_list.resize(n_shapes);
		for(int i = 0; i < n_shapes; i++)
			InitShapeFromDetBox(shape_list[i], detected_list[i], fit_asm.GetMappingDetShape(), fit_asm.GetMeanFaceWidth());		

		IplImage ipl_img = frame;
		bool flag = fit_asm.ASMSeqSearch(shape_list[0], &ipl_img, fr_asm_frame, true, n_iterations);
		if(flag) fit_asm.Draw(&ipl_img, shape_list[0]);

		int n_pts = shape_list[0].NPoints();
		facePoints.resize(n_pts);
		char buff[10];
		for (int i=0;i<n_pts;i++)
		{
			const Point2D32f& pt = shape_list[0][i];
			facePoints[i].x = pt.x;
			facePoints[i].y = pt.y;
			vDrawText(&ipl_img, pt.x, pt.y, itoa(i, buff, 10));
		}

#ifdef _DEBUG
		show_mat(frame);
		cv::waitKey(1);
#endif
		//why i use temp_tri ? for sake of multi-thread latency time
		std::vector<TriangleData> temp_tri = ciTri::triangulate(facePoints, facePoints.size(), false);

		LOCK_START(mtx_triList);
		face_indices = temp_tri;
		face_vertices = facePoints;
		modifyVertices(face_vertices);
		face_vertices2 = facePoints;
		LOCK_END();

		runASMThread.yield();
	}
}

void FaceOffApp::keyDown( KeyEvent event )
{
	int code = event.getCode();
	if (code == event.KEY_ESCAPE)
		quit();
	if (code == event.KEY_SPACE)
		faceFrameVisible = !faceFrameVisible;
}

void FaceOffApp::modifyVertices(std::vector<ci::Vec2f>& vertices)
{
#define BIG_EYE_MOUTH
#ifdef BIG_EYE_MOUTH
	//eyes
	static int left_eye_idx[] = {27,28,29,30};
	static int right_eye_idx[] = {32,33,34,35};
	left_eye.init(vertices, 31, left_eye_idx, 4);
	right_eye.init(vertices, 36, right_eye_idx, 4);

	float k = 2.5;
	left_eye.scale(ci::Vec2f(k*0.9,k));
	right_eye.scale(ci::Vec2f(k*0.9,k));

	//mouth
	vector<int> mouth_idx;
	for (int idx=48;idx<=65;idx++)
		mouth_idx.push_back(idx);
	mouth.init(vertices, 66, &mouth_idx[0], mouth_idx.size());
	k = 1.5;
	mouth.scale(ci::Vec2f(k,k));
#endif
}

CINDER_APP_BASIC( FaceOffApp, RendererGl )
