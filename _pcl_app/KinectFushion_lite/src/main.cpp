#include "Fushion.h"

int main(int argc, char** argv)
{
	KinectFushionApp fushion;
	if (!fushion.setup())
		return -1;

	while (true)
	{
		int key = cv::waitKey(1);
		if (key == VK_ESCAPE)
			break;
	}
	fushion.exit = true;

 	return 0;
}