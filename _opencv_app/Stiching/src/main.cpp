#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include "../../../_common/ofxDirList.h"

#if defined _DEBUG
#pragma comment(lib,"opencv_core232d.lib")
#pragma comment(lib,"opencv_imgproc232d.lib")
#pragma comment(lib,"opencv_highgui232d.lib")
#pragma comment(lib,"opencv_stitching232d.lib")
#else
#pragma comment(lib,"opencv_core232.lib")
#pragma comment(lib,"opencv_imgproc232.lib")
#pragma comment(lib,"opencv_highgui232.lib")
#pragma comment(lib,"opencv_stitching232.lib")
#endif

using namespace std;
using namespace cv;

bool try_use_gpu = false;
std::vector<Mat> imgs;
std::string result_name = "result.jpg";

int main(int argc, char* argv[])
{
	ofxDirList DIR;
	DIR.allowExt("png");
	DIR.allowExt("jpg");
	int n_images = DIR.listDir("../media/tsucuba");
	for (int i=0;i<n_images;i++)
	{
		Mat img = imread(DIR.getPath(i));
		if (!img.empty())
			imgs.push_back(img);
	}

    Mat pano;
    Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
    Stitcher::Status status = stitcher.stitch(imgs, pano);

    if (status != Stitcher::OK)
    {
        cout << "Can't stitch images, error code = " << status << endl;
        return -1;
    }

//    imwrite(result_name, pano);
	imshow("result", pano);

	waitKey();

    return 0;
}