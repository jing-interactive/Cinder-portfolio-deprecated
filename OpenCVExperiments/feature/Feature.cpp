#if defined _DEBUG
#pragma comment(lib,"opencv_core231d.lib")
#pragma comment(lib,"opencv_imgproc231d.lib")
#pragma comment(lib,"opencv_highgui231d.lib")
#pragma comment(lib,"opencv_objdetect231d.lib")
#pragma comment(lib,"opencv_features2d231d.lib")
#pragma comment(lib,"opencv_flann231d.lib")
#else
#pragma comment(lib,"opencv_core231.lib")
#pragma comment(lib,"opencv_imgproc231.lib")
#pragma comment(lib,"opencv_highgui231.lib")
#pragma comment(lib,"opencv_objdetect231.lib")
#pragma comment(lib,"opencv_features2d231.lib")
#pragma comment(lib,"opencv_flann231.lib")
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
	"{2|||the right picature to do comparison}"
	"{faceDetect|face|false|do face detection first}" 
	"{equalizeHist|eq|false|call cv::equalizeHist first}"
	"{v|visual|false|whether to visualizing the result}"
	"{h|help|false|display this help text}"
	"{verb|verbose|false|be noisy}"
};

int k_min = 5;
int k_max = 55;
bool new_frame = true;

void onTracker(int pos, void* userdata)
{
	new_frame = true;
}

int main(int argc, const char** argv)
{
	cv::CommandLineParser args(argc, argv, keys);
	if(argc < 3)
	{
		help();
		args.printParams();
		return -1;
	}

	Mat img1,img2;
	std::string filename1(argv[1]),filename2(argv[2]);

	Mat src1 = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	Mat src2 = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	if(src1.empty() || src2.empty())
	{
		if (args.get<bool>("verbose"))
			printf("Can't read one of the images\n");
		return -1;
	}

	if (args.get<bool>("faceDetect"))
	{
		cv::CascadeClassifier mFaceCascade;
		mFaceCascade.load("haarcascade_frontalface_alt.xml");
		vector<cv::Rect> faces;

		mFaceCascade.detectMultiScale(src1, faces);
		if (!faces.empty())
			src1(faces[0]).copyTo(img1);
		else
			src1.copyTo(img1);

		mFaceCascade.detectMultiScale(src2, faces);
		if (!faces.empty())
			src2(faces[0]).copyTo(img2);
		else
			src2.copyTo(img2);
	}
	else
	{
		src1.copyTo(img1);
		src2.copyTo(img2);
	}

	if (args.get<bool>("equalizeHist"))
	{
		cv::equalizeHist(img1, img1);
		cv::equalizeHist(img2, img2);
	}

	if (args.get<bool>("visual"))
	{
		namedWindow("feature", 0);
		createTrackbar("min_dist", "feature", &k_min, 100, onTracker);
		createTrackbar("max_dist", "feature", &k_max, 100, onTracker);
	}

	// detecting keypoints
	Ptr<FeatureDetector> detector = new SiftFeatureDetector;
	vector<KeyPoint> keypoints1, keypoints2;
	detector->detect(img1, keypoints1);
	detector->detect(img2, keypoints2);

	// computing descriptors
	Ptr<DescriptorExtractor> extractor = new SiftDescriptorExtractor;
	Mat descriptors1, descriptors2;
	extractor->compute(img1, keypoints1, descriptors1);
	extractor->compute(img2, keypoints2, descriptors2);

	// matching descriptors
	Ptr<DescriptorMatcher> matcher;
	vector<DMatch> matches;
#if 0
	matcher = new BruteForceMatcher<L2<float> >;	
	matcher->match(descriptors1, descriptors2, matches);
#else
	matcher = new FlannBasedMatcher;
	matcher->match( descriptors1, descriptors2, matches );
#endif
#if 1
	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors1.rows; i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

#endif

	Mat img_matches;
	std::vector< DMatch > good_matches;

	if (args.get<bool>("visual"))
	{
		printf("-- Max dist : %f \n", max_dist );
		printf("-- Min dist : %f \n", min_dist );

		while (true)
		{
			if (new_frame)
			{
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
				drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), 
					vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
				imshow("feature", img_matches);
				new_frame = false;
			}
			int key = waitKey(1);
			if (key == 0x1B)
				break;
		}
	}
	else
	{
		for( int i = 0; i < descriptors1.rows; i++ )
		{ 
			float dist = matches[i].distance;
			if( dist >= min_dist*k_min*0.01 && dist <= max_dist*k_max*0.01)
			{
				good_matches.push_back( matches[i]);
			}
		}
	}

	if (args.get<bool>("verbose"))
	{
		printf("[%s] [%s] %d matches.\n", 
			filename1.c_str(), filename2.c_str(), good_matches.size());
	}

	return good_matches.size();
}
