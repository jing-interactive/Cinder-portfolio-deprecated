#include <opencv2/opencv.hpp>
#include "gl_helper.h"
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

int k_min = 5;
int k_max = 55;

class FeatureMatcher
{
public:
	FeatureMatcher(string detectorType = "SURF", string extractorType="SURF", string matcherType="FlannBased"):
	  _detectorType(detectorType),_extractorType(extractorType),_matcherType(matcherType){
		  _detector = FeatureDetector::create(detectorType);
		  _extractor = DescriptorExtractor::create(extractorType);
		  _matcher = DescriptorMatcher::create(matcherType);
	  }

	  bool setObjectImage(const Mat& src)
	  {
		  _img1 = src;
		  return _calculate(src, keypoints1, descriptors1);
	  }

	  bool setSceneImage(const Mat& src)
	  {
		  _img2 = src;
		  if (!_calculate(src, keypoints2, descriptors2))
			  return false;
		  _matcher->match( descriptors1, descriptors2, matches);
#if 1
		  max_dist = 0; 
		  min_dist = 100;

		  //-- Quick calculation of max and min distances between keypoints
		  for( int i = 0; i < descriptors1.rows; i++ )
		  { 
			  double dist = matches[i].distance;
			  if( dist < min_dist ) min_dist = dist;
			  if( dist > max_dist ) max_dist = dist;
		  }
#endif
		  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
		  //-- PS.- radiusMatch can also be used here.
		  good_matches.clear();

		  for( int i = 0; i < descriptors1.rows; i++ )
		  { 
			  float dist = matches[i].distance;
			  if( dist >= min_dist*k_min*0.01 && dist <= max_dist*k_max*0.01)
			  {
				  good_matches.push_back( matches[i]);
			  }
		  }
		  return true;
	  }

	  Mat findHomography(int ransacThresh = 3)
	  {
		  vector<Point2f> pt1,pt2;

		  int n_matches = good_matches.size();
		  for(int i = 0; i < n_matches; i++){
			  pt1.push_back(keypoints1[good_matches[i].queryIdx].pt);
			  pt2.push_back(keypoints2[good_matches[i].trainIdx].pt);
		  }
		  return cv::findHomography(pt1, pt2, Mat(), CV_RANSAC, ransacThresh);
	  }

	  void draw(Mat& match_img)
	  {
		  drawMatches(_img1, keypoints1, _img2, keypoints2, good_matches, match_img, Scalar::all(-1), Scalar::all(-1), 
			  vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	  }

private:
	bool _calculate(const Mat& src, vector<KeyPoint>& pts, Mat& descr)
	{
		_detector->detect(src, pts);
		_extractor->compute(src, pts, descr);

		return pts.size() > 0;
	}

	string _detectorType;
	string _extractorType;
	string _matcherType;

	Ptr<FeatureDetector> _detector;
	Ptr<DescriptorExtractor> _extractor;
	Ptr<DescriptorMatcher> _matcher;

	vector<KeyPoint> keypoints1,keypoints2;
	Mat descriptors1, descriptors2;
	vector<DMatch> matches;	
	vector<DMatch> good_matches;

	double max_dist, min_dist;

	Mat _img1,_img2;
};

bool new_frame = true;

void onTracker(int pos, void* userdata)
{
	new_frame = true;
}

#define OPENGL_TITLE "opengl"


struct ThisGl : public I3DRenderer
{
	ThisGl():I3DRenderer(OPENGL_TITLE){}

	void update()
	{
		double aspect = getWindowProperty(_title, WND_PROP_ASPECT_RATIO);
		_camera.setPerspectiveProjection(45, aspect, 0.1, 1000.0);
		_camera.lookAt(Point3d(0,0,-1), Point3d(0,0,0), Point3d(0,1,0));
	}

	void draw()
	{
		_camera.setupProjectionMatrix();
		_camera.setupModelViewMatrix();
 
		glutWireTeapot(2);

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

	Ptr<FeatureMatcher> feature = new FeatureMatcher(detectorType, extractorType, matcherType);
	feature->setObjectImage(img_object);

	namedWindow("match.param", 0);
	{		
		createTrackbar("min_dist", "match.param", &k_min, 100, onTracker);
		createTrackbar("max_dist", "match.param", &k_max, 100, onTracker);
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
		bool ok = feature->setSceneImage(gray);
		if (ok)
		{
			feature->draw(img_matches);

			Mat homo = feature->findHomography(5);

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
		}
		//imshow("camera", frame);

		renderer.update();
		{
			renderer._bg_tex.copyFrom(frame,false);
		}
		updateWindow(OPENGL_TITLE);

		int key = waitKey(1);
		if (key == 0x1B)
			break;
	}
}
