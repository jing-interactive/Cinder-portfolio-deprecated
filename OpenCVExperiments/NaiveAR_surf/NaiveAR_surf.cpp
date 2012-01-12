#if defined _DEBUG
#pragma comment(lib,"opencv_core232d.lib")
#pragma comment(lib,"opencv_imgproc232d.lib")
#pragma comment(lib,"opencv_highgui232d.lib")
#pragma comment(lib,"opencv_objdetect232d.lib")
#pragma comment(lib,"opencv_features2d232d.lib")
#pragma comment(lib,"opencv_flann232d.lib")
#else
#pragma comment(lib,"opencv_core232.lib")
#pragma comment(lib,"opencv_imgproc232.lib")
#pragma comment(lib,"opencv_highgui232.lib")
#pragma comment(lib,"opencv_objdetect232.lib")
#pragma comment(lib,"opencv_features2d232.lib")
#pragma comment(lib,"opencv_flann232.lib")
#endif

#include <opencv2/opencv.hpp>

using namespace cv;

void help()
{
	printf("\nThis program demonstrates using features2d detector, descriptor extractor and simple matcher\n"
		"Using the SURF desriptor:\n"
		"\n"
		"Usage:\n matcher_simple <image1> <image2>\n");
}

const char *keys =
{
	"{1|||the left picature to do comparison}"
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

	  void setReferenceImage(const Mat& src)
	  {
		  _calculate(src, keypoints1, descriptors1);
	  }

	  void setTestImage(const Mat& src)
	  {
		  _calculate(src, keypoints2, descriptors2);
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
	  }
	  void draw(Mat& match_img)
	  {
		  drawMatches(_img1, keypoints1, _img2, keypoints2, good_matches, match_img, Scalar::all(-1), Scalar::all(-1), 
			  vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	  }

private:
	void _calculate(const Mat& src, vector<KeyPoint>& pts, Mat& descr)
	{
		_detector->detect(src, pts);
		_extractor->compute(src, pts, descr);
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

int main(int argc, const char** argv)
{
	cv::CommandLineParser args(argc, argv, keys);

	Mat ref_img = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	if(ref_img.empty())
	{
		printf("Can't read the reference image\n");
		return -1;
	}

	string detectorType = args.get<std::string>("detector");
	string extractorType = args.get<std::string>("extractor");
	string matcherType = args.get<std::string>("matcher");

	Ptr<FeatureMatcher> feature = new FeatureMatcher(detectorType, extractorType, matcherType);
	feature->setReferenceImage(ref_img);

	Mat img_matches;	

	VideoCapture input(0);
	if (!input.isOpened())
		printf("camera not opened\n");
	Mat frame,gray;
	Mat match_img;

	while (true)
	{
		input >> frame;
		cvtColor(frame, gray, CV_BGR2GRAY);
		feature->setTestImage(gray);
		feature->draw(match_img);

		imshow("match", match_img);

		int key = waitKey(1);
		if (key == 0x1B)
			break;
	}
}
