#include "../../../_common/pcl/pcl.h"
#include <pcl/io/ply_io.h> 
#include <opencv2/opencv.hpp>
#include "../../../_common/vOpenCV/OpenGL.h"
#include "../../../_common/pcl/kinect_support.h"

using namespace cv;
using namespace pcl;

int main(int argc, char** argv)
{
	PCL_INFO("Syntax is %s [pcd file]\n", argv[0]);

	PointCloudPtr raw(new PointCloudT);
	std::vector<int> pcd_indices;
	pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	
	if (!pcd_indices.empty())
	{
		char* input_pcd = argv[pcd_indices[0]];
		PCL_INFO ("Reading %s\n",input_pcd);		
		int r = io::loadPCDFile(input_pcd, *raw);
		if (r <0)
		{
			PCL_ERROR("Please specify a valid pcd file\n");
			return -1;
		}
	}
	else
	{
		PCL_ERROR("Please specify a pcd file\n");
		return -1;
	}

	Ptr<PointCloudViewer> renderer = NEW_CLOUD_VIEWER(raw);

	while (true)
	{
		renderer->update();

		int key = waitKey(1);
		if (key == VK_ESCAPE)
			break;
	}

 	return 0;
}