#include "../../../_common/vOpenCV/OpenCV.h"
#include "../../../_common/vOpenCV/BlobTracker.h"
#include <set>

using namespace cv;

enum PointState{
	NEAR_LEFT,
	NEAR_RIGHT,
	NEAR_TOP,
	NEAR_BOTTOM,
	NEAR_NOTHING,
};

PointState getPointState(const Point& pt, int width, int height, int id)
{
	int x0 = width*id;
	int x1 = width*(id+1);
	int y0 = 0;
	int y1 = height;
	const int thresh = 3;
	if (pt.x < x0+thresh)
		return NEAR_LEFT;
	if (pt.x > x1-thresh)
		return NEAR_LEFT;
	if (pt.y < thresh)
		return NEAR_TOP;
	if (pt.y > height-thresh)
		return NEAR_BOTTOM;
	return NEAR_NOTHING;
}

int main(int argc, char** argv )
{	
	Mat frame = cvLoadImage("../media/scene.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	int W = frame.cols;
	int H = frame.rows;

	vector<vBlob> blobs;
	vThreshInv(frame, 100);
	vClose(frame,2);

	show_mat(frame);
	cvWaitKey();
	vFindBlobs(frame, blobs,100);
	frame = 0; 
	int min_idx	= -1;
	for (int i=0;i<blobs.size();i++)
	{
		printf("\n");
		std::set<PointState> NearPointSet; 

		vBlob& b = blobs[i];
		vPolyLine(frame, b.pts);

		Point ptx(W/2, H/2);
		for (int j=0;j<b.pts.size();j++)
		{
			PointState st = getPointState(b.pts[j], W, H, 0);
			if (st != NEAR_NOTHING)
			{
				NearPointSet.insert(st);
				ptx = b.pts[j];
				break;
			}
		}

		bool only_one_near = NearPointSet.size() == 1;
		int min_value = only_one_near ? 0 : INT_MAX;
		for (int j=0;j<b.pts.size();j++)
		{
			Point diff = b.pts[j] - ptx;
			float dist = norm(diff);

			if (
				(only_one_near && dist > min_value) ||
				(!only_one_near && dist < min_value)
				)
			{
				min_value = dist;
				min_idx = j;
			}
		}

		int n_neighbors = 1;
		float sum_x = b.pts[min_idx].x;
		float sum_y = b.pts[min_idx].y;
		for (int j=0;j<b.pts.size();j++)
		{
			Point diff = b.pts[j] - b.pts[min_idx];
			if (norm(diff) < 100)
			{
				sum_x += b.pts[j].x;
				sum_y += b.pts[j].y;
				n_neighbors++;
			}
		}
		sum_x /= n_neighbors;
		sum_y /= n_neighbors;
		circle(frame, b.pts[min_idx], 10, CV_WHITE,-1);
		circle(frame, Point(sum_x, sum_y), 5, CV_WHITE,-1);
	}

	show_mat(frame);
	waitKey(0); 

	return 0;
}