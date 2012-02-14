#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <functional>

#if defined _DEBUG
#pragma comment(lib,"opencv_core232d.lib")
#pragma comment(lib,"opencv_imgproc232d.lib")
#pragma comment(lib,"opencv_highgui232d.lib")
#pragma comment(lib,"opencv_features2d232d.lib")
#else
#pragma comment(lib,"opencv_core232.lib")
#pragma comment(lib,"opencv_imgproc232.lib")
#pragma comment(lib,"opencv_highgui232.lib")
#pragma comment(lib,"opencv_features2d232.lib")
#endif

using namespace cv;
using namespace std;

#define W 640
#define H 480

void randomize(Point& p)
{
	p.x = rand()%W;
	p.y = rand()%H;
}
 
struct draw
{
	draw(Mat& frame, const Scalar& clr, const vector<Point>& train, const vector<Point>& query):
		_frame(frame),_clr(clr),_train(train),_query(query)
	{

	}
	void operator()(Point& p)
	{
		circle(_frame, p, 3, _clr);
	}
	void operator()(DMatch& match)
	{
		int t_id = match.trainIdx;
		int q_id = match.queryIdx;
		line(_frame, _train[t_id], _query[q_id], _clr);
	}
	void operator()(vector<DMatch>& matches)
	{
		for_each(matches.begin(), matches.end(), draw(_frame, _clr, _train, _query));
	}
	Mat& _frame;
	const Scalar& _clr;
	const vector<Point>& _train;
	const vector<Point>& _query;
};

int main(int argc, char** argv )
{	
	Mat frame(H,W,CV_8UC3);
	vector<Point> pta(20);
	vector<Point> ptb(24);
	vector<int> nn_of_a(pta.size());//nearest neighbor of pta in ptb
	vector<float> dist_of_a(pta.size());//nearest neighbor of pta in ptb

	BruteForceMatcher<L2<int> > matcher;
	vector<DMatch> matches;
	for_each(pta.begin(), pta.end(), randomize);
	namedWindow("out");

#define RADIUS 100

	while (true)
	{
		frame = Scalar::all(255);

		for_each(ptb.begin(), ptb.end(), randomize);

		for_each(pta.begin(), pta.end(), draw(frame, Scalar(255,0,0), pta, ptb));
		for_each(ptb.begin(), ptb.end(), draw(frame, Scalar(0,0,255), pta, ptb));

		cv::Mat ma(pta.size(), 2, CV_32SC1, (void*)&pta[0]);
		cv::Mat mb(ptb.size(), 2, CV_32SC1, (void*)&ptb[0]);

// 		cout<<ma<<endl;
// 		cout<<mb<<endl; 

		matcher.match(mb, ma, matches);
		fill(nn_of_a.begin(), nn_of_a.end(),-1);
		fill(dist_of_a.begin(), dist_of_a.end(),FLT_MAX);

		for (int i=0;i<matches.size();i++)
		{
			const DMatch& match = matches[i];
			int t_id = match.trainIdx;
			int q_id = match.queryIdx;
			float dist = match.distance;

			if (dist < RADIUS && dist < dist_of_a[t_id])
			{
				dist_of_a[t_id] = dist;
				nn_of_a[t_id] = q_id;
			}
		}
//		for_each(matches.begin(), matches.end(), draw(frame, Scalar::all(0), pta, ptb));
		for (int i=0;i<nn_of_a.size();i++)
		{
			int nn = nn_of_a[i];
			if (nn != -1)
				line(frame, pta[i], ptb[nn], Scalar::all(0));
		}

		imshow("out", frame);
		if (cv::waitKey() == VK_ESCAPE)
			break;
		copy(ptb.begin(),ptb.begin()+pta.size(),pta.begin());
	}

	return 0;
}
