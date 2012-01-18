#include <opencv2/opencv.hpp>
#include "../../../_common/vOpenCV/OpenGL.h"
#include "../../../_common/vOpenCV/FeatureMatcher.h"
#include <gl/glut.h>

using namespace cv;

const char *keys =
{
	"{input|i|../media/mie_3.jpg|the left picature to do comparison}"
	"{detector|d|SURF|could be SIFT/SURF/ORB/FAST/STAR//MSER/GFTT/HARRIS/Dense/SimpleBlob/Grid/Pyramid/Dynamic}" 
	"{extractor|e|SURF|could be SIFT/SURF/ORB/BRIEF/Opponent}"
	"{matcher|e|FlannBased|could be FlannBased/BruteForce/BruteForce-SL2/BruteForce-L1/BruteForce-Hamming/"
	"BruteForce-HammingLUT/BruteForce-Hamming(2)/BruteForce-Hamming(4)}"
	"{h|help|false|display this help text}"
	"{verb|verbose|false|be noisy}"
};

bool new_frame = true;

void onTracker(int pos, void* userdata)
{
	new_frame = true;
}

#define OPENGL_TITLE "opengl"

Ptr<FeatureMatcher> the_matcher;

struct ThisGl : public I3DRenderer
{
	ThisGl():I3DRenderer(OPENGL_TITLE){}

	void draw()
	{
		_camera.lookAt(Point3d(0,0,-10), Point3d(0,0,0), Point3d(0,1,0));
		_camera.setupProjectionMatrix();
		//_camera.setupModelViewMatrix();

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glPushMatrix();
		if (!the_matcher->_ModelView.empty())
		{
			glMultMatrixd(the_matcher->_ModelView.ptr<double>());
		}

		glutWireTeapot(1);
		glPopMatrix();
		//	render(_bg_tex);
	}
	GlTexture _bg_tex;
};

int main(int argc, const char** argv)
{
	cv::CommandLineParser args(argc, argv, keys);

	Mat img_object = imread(args.get<std::string>("input"), CV_LOAD_IMAGE_GRAYSCALE);
	if(img_object.empty())
	{
		printf("Can't read the reference image\n");
		return -1;
	}

	string detectorType = args.get<std::string>("detector");
	string extractorType = args.get<std::string>("extractor");
	string matcherType = args.get<std::string>("matcher");

	the_matcher = new FeatureMatcher(detectorType, extractorType, matcherType);
	the_matcher->setObjectImage(img_object);

	namedWindow("match.param", 0);
	{		
		createTrackbar("min_dist", "match.param", &the_matcher->k_min, 100, onTracker);
		createTrackbar("max_dist", "match.param", &the_matcher->k_max, 100, onTracker);
	}

	ThisGl renderer;

	Mat img_matches;	

	VideoCapture input(0);
	if (!input.isOpened())
		printf("camera not opened\n");
	Mat frame,gray; 

	while (true)
	{
		input >> frame;
		cvtColor(frame, gray, CV_BGR2GRAY);
		bool ok = the_matcher->setSceneImage(gray);
		if (ok)
		{
			the_matcher->draw(img_matches);

			Mat homo = the_matcher->findHomography(5);

			//-- Get the corners from the image_1 ( the object to be "detected" )
			std::vector<Point2f> obj_corners(4);
			obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
			obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
			std::vector<Point2f> scene_corners(4);

			perspectiveTransform( obj_corners, scene_corners, homo);

			//-- Draw lines between the corners (the mapped object in the scene - image_2 )
			line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
			line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
			line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
			line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

			imshow("match", img_matches);

			the_matcher->solve2();
//			the_matcher->solvePnp();
		}
		//imshow("camera", frame);

		//		renderer.update();
		{
			//	renderer._bg_tex.copyFrom(frame,false);
		}
		updateWindow(OPENGL_TITLE);

		int key = waitKey(1);
		if (key == 0x1B)
			break;
	}
}
