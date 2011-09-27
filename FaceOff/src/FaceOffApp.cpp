#include "cinder/app/AppBasic.h"
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

class FaceOffApp : public AppBasic {
public:
	//cinder routines
	void setup();
	void shutdown();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();

	//input
	void grabCamera();
	std::thread grabCameraThread;
	Capture		mCap;
	Surface		surInput;
	CSimpleMutex mtx_surInput;

	//process
	void detectFace();
	std::thread detectFaceThread;
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
	CSimpleMutex				mtx_triList;

	//rendering
	gl::Texture		tex_frame;
	bool running;
	ci::TriMesh triMesh;
	gl::VboMesh vboMesh;
};

void FaceOffApp::setup()
{
	running = true;
	try {
		mCap = Capture( 640, 480 );
		mCap.start();
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
	triMesh.clear();
	for (int i=0;i<face_vertices.size();i++)
	{
		triMesh.appendVertex(ci::Vec3f(face_vertices[i],0));
		triMesh.appendColorRGB( Color( 0.1, 0.7, 0.5 ) );
	}
	for (int i=0;i<face_indices.size();i++)
	{
		const TriangleData& tri = face_indices[i];
		triMesh.appendTriangle(tri.idx[0], tri.idx[1], tri.idx[2]);
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

	gl::color( Color( 1, 1, 1) );
	gl::draw(tex_frame);
	tex_frame.disable();

	// draw the faces as transparent yellow rectangles
	gl::color( ColorA( 1, 1, 0, 0.25f ) );
	LOCK_START(mtx_mFaces);
	for (int i=1;i<mFaces.size();i++) //<--- i begins with 1
	{
		const Rectf& r = mFaces[i];
		gl::drawSolidCircle( r.getCenter(),  (r.getWidth()+r.getHeight())/4);
	}
	LOCK_END();

	if (triMesh.hasColorsRGB())//test if..
	{
		gl::enableWireframe();
		gl::draw(triMesh);
		gl::disableWireframe();
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

#ifdef _DEBUG
		show_mat(frame);
		cv::waitKey(1);
#endif
		int n_pts = shape_list[0].NPoints();
		facePoints.resize(n_pts);
		for (int i=0;i<n_pts;i++)
		{
			const Point2D32f& pt = shape_list[0][i];
			facePoints[i].x = pt.x;
			facePoints[i].y = pt.y;
		}

		//why i use temp_tri ? for sake of multi-thread latency time
		std::vector<TriangleData> temp_tri = ciTri::triangulate(facePoints, facePoints.size(), false);

		LOCK_START(mtx_triList);
		face_indices = temp_tri;
		face_vertices = facePoints;
		LOCK_END();

		runASMThread.yield();
	}
}

CINDER_APP_BASIC( FaceOffApp, RendererGl )
