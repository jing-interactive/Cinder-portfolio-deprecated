#include "../../../_common/vOpenCV/OpenCV.h"
#include "../../../_common/vOpenCV/BlobTracker.h"

VideoInput input;

using namespace cv;

int main(int argc, char **argv)
{
	if (argc != 3)
	{
		fprintf(stdout, "usage: %s inputImageName threshNum\n", argv[0]);
		return -1;
	}
	string name(argv[1]);
	int thresh = atoi(argv[2]);

	Mat3b src = cvLoadImage(argv[1]);
	show_mat(src);

	Mat3b frame = src.clone();
	floodFill(frame, Point(1,1),Scalar(0,0,0), NULL, Scalar::all(thresh), Scalar::all(thresh));
	show_mat(frame);

	Mat1b gray;
	vGrayScale(frame, gray);

	// Create mat with alpha channel
	cv::Mat4b dst(src.size());

	for (int j = 0; j < dst.rows; ++j) 
	{
		for (int i = 0; i < dst.cols; ++i)
		{
			cv::Vec4b& rgba = dst(j, i);
			cv::Vec3b& rgb = src(j, i); 
			rgba[0] = rgb[0];
			rgba[1] = rgb[1];
			rgba[2] = rgb[2];
			if (gray(j,i) > 0)
				rgba[3] = 255;
			else
				rgba[3] = 0;
		}
	}

	try {
		std::vector<int> params;
		params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		params.push_back(9);
		cvSaveImage(string(name+".png").c_str(), &(IplImage)dst, &params[0]);
	}
	catch (std::runtime_error& ex) {
		fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
		return 1;
	}

	fprintf(stdout, "Saved PNG file with alpha data.\n");

	waitKey();
}
