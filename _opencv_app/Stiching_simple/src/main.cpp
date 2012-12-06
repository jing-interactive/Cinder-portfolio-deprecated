#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include "../../../_common/ofxDirList.h"

#if defined _DEBUG
#pragma comment(lib,"opencv_core243d.lib")
#pragma comment(lib,"opencv_imgproc243d.lib")
#pragma comment(lib,"opencv_highgui243d.lib")
#pragma comment(lib,"opencv_stitching243d.lib")
#else
#pragma comment(lib,"opencv_core243.lib")
#pragma comment(lib,"opencv_imgproc243.lib")
#pragma comment(lib,"opencv_highgui243.lib")
#pragma comment(lib,"opencv_stitching243.lib")
#endif

using namespace std;
using namespace cv;

bool try_use_gpu = false;
std::vector<Mat> imgs;
std::string result_name = "result.png";

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("usage: %s path/folder/contains/images the_output_image=\"result.png\" \n", argv[0]);
		return -1;
	}

	string folder = argv[1];
	if (argc == 3)
		result_name = argv[2];
	
	ofxDirList DIR;
	DIR.allowExt("png");
	DIR.allowExt("jpg");
	int n_images = DIR.listDir(folder);
	for (int i=0;i<n_images;i++)
	{
		Mat raw = imread(DIR.getPath(i));
		if (!raw.empty())
		{
			imgs.push_back(raw);
		}
	}

    Mat pano;
    Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
    Stitcher::Status status = stitcher.stitch(imgs, pano);

    if (status != Stitcher::OK)
    {
        cout << "Can't stitch images, error code = " << status << endl;
        return -1;
    }

    imwrite(result_name, pano);
	imshow(result_name, pano);

	waitKey();

    return 0;
}