#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

#if defined _DEBUG
#pragma comment(lib,"opencv_core231d.lib")
#pragma comment(lib,"opencv_imgproc231d.lib")
#pragma comment(lib,"opencv_highgui231d.lib")
#pragma comment(lib,"opencv_gpu231d.lib")
#else
#pragma comment(lib,"opencv_core231.lib")
#pragma comment(lib,"opencv_imgproc231.lib")
#pragma comment(lib,"opencv_highgui231.lib")
#pragma comment(lib,"opencv_gpu231.lib")
#endif

using namespace cv;

#define SHOW_MAT(mat) do{namedWindow(#mat);imshow(#mat,mat);}while(0)
static Mat temp_mat;
#define SHOW_GPUMAT(mat) do{namedWindow(#mat##"<gpu>");temp_mat = mat;imshow(#mat##"<gpu>",temp_mat);}while(0)

void highpass(const gpu::GpuMat& src, gpu::GpuMat& dst, int blur1, int blur2 ) 
{
	gpu::GpuMat temp;
	if(blur1 > 0)
		gpu::blur(src, temp, Size((blur1*2)+1, (blur1*2)+1));
	//Original Image - Blur Image = Highpass Image
	gpu::subtract( src, temp, dst );
	int col = temp_mat.cols;
	int row = temp_mat.rows;

	//Blur Highpass to remove noise
	if(blur2 > 0)
		gpu::blur(temp, dst, Size((blur2*2)+1, (blur2*2)+1));
}

void highpass(const Mat& src, Mat& dst, int blur1, int blur2 ) 
{
	if(blur1 > 0)
		cv::blur(src, dst, Size((blur1*2)+1, (blur1*2)+1));

	//Original Image - Blur Image = Highpass Image
	cv::subtract( src, dst, dst );
	SHOW_MAT(dst);

	//Blur Highpass to remove noise
	if(blur2 > 0)
		cv::blur(dst, dst, Size((blur2*2)+1, (blur2*2)+1));
}

int blur1 = 20;
int blur2 = 10;

#include <windows.h>

int main (int argc, char* argv[])
{
	int nDevices = gpu::getCudaEnabledDeviceCount();
	if (nDevices > 0)
	{
		gpu::DeviceInfo info(0);
		printf("%s, Version %d.%d\n", info.name().c_str(),info.majorVersion(),info.minorVersion());
	}

	namedWindow("main");
	createTrackbar("blur1", "main", &blur1, 100);
	createTrackbar("blur2", "main", &blur2, 100);

	Mat img1 = imread("fore.jpg",0);
	Mat img2 = imread("back.jpg",0);
	gpu::GpuMat dst, src1,src2;
	src1 = img1;
	src2 = img2;

	while (true)
	{
#if 1
		DWORD time = GetTickCount();
		gpu::subtract(src1, src2, src2);
		DWORD sub = (GetTickCount() - time);
		gpu::threshold(src2, src1, 50, 255.0, CV_THRESH_BINARY);
		DWORD thr = (GetTickCount() - time);
		highpass(src1, dst,blur1, blur2);
		DWORD high = (GetTickCount() - time);
		SHOW_GPUMAT(dst);
#else
		DWORD time = GetTickCount();
		cv::subtract(img1, img2, img2);
		DWORD sub = (GetTickCount() - time);
		cv::threshold(img2, img1, 50, 255.0, CV_THRESH_BINARY);
		DWORD thr = (GetTickCount() - time);
		highpass(img1, img2,blur1, blur2);
		DWORD high = (GetTickCount() - time);
		SHOW_MAT(img2);
#endif
		int key = waitKey(1);
		if (key == VK_ESCAPE)
			break;
		DWORD total = (GetTickCount() - time);

		printf("%d %d %d %d ms\n", sub, thr, high, total);
	}
	return 0;
}
