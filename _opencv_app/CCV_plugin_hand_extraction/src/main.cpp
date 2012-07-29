#include "../../../_common/vOpenCV/OpenCV.h"
#include "../../../_common/vOpenCV/BlobTracker.h"

using namespace cv;

int main(int argc, char** argv )
{	
	Mat frame = cvLoadImage("../media/scene.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	
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
		int min_value = INT_MAX;
		vBlob& b = blobs[i];
		vPolyLine(frame, b.pts);
		for (int j=0;j<b.pts.size();j++)
		{
			Point diff = b.pts[j] - Point(400, 300);
			float dist = norm(diff);
			if (dist < min_value)
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