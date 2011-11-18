#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/console/parse.h>

using namespace pcl;

void randomize_cloud(PointCloud<PointXYZ>& cloud_)
{
	const int k = 80;
	for (int x=-k; x<k; x++) {
		for (int y=-k; y<k; y++) {
			cloud_.push_back( PointXYZ(x, y, sin(x/10.0f)*cos(x/5.0f+y/20.0f)*k/4) );
		}
	}
}

int main(int argc, char** argv)
{
	PCL_INFO("Syntax is %s [-i pcd file]\n", argv[0]);

	std::string pcd_file;
	std::string ply_file("random.ply");

	PointCloud<PointXYZ> cloud;
	if (console::parse_argument (argc, argv, "-i", pcd_file) > 0)
	{
		PCL_INFO ("Reading %s\n",pcd_file.c_str());		
		io::loadPCDFile(pcd_file, cloud);
		ply_file = pcd_file + ".ply";
	}
	else
	{
		PCL_INFO("No pcd file specified, a random point cloud model will be generated\n");
		randomize_cloud(cloud);
	}
	//write 
	PCL_INFO("Writing %s\n",ply_file.c_str());
	io::savePLYFileASCII(ply_file, cloud);

	return 0;
}