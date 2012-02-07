#include "../../../_common/vOpenCV/OpenCV.h"
#include "../../../_common/vOpenCV/BlobTracker.h"

VideoInput input;

using namespace cv;

int main(int argc, char** argv )
{	
	if (input.init(argc,argv))
	{
		vHaarFinder face;
		if (!face.init("../media/haarcascade_frontalface_alt.xml"))
			return -1;
		face.scale = 1.5;

		while (true)
		{
			IplImage* _raw = input.get_frame(); 
			if (!_raw)
				break;
			Mat raw = _raw;
			face.find(&(IplImage)raw);
			int n_faces = face.blobs.size();
			for (int i=0;i<n_faces;i++)
			{
			//	cv::rectangle(raw, face.blobs[i].box, vDefaultColor(i));
				vDrawText(raw, face.blobs[i].center.x,face.blobs[i].box.y - 10, "Hello world!", vDefaultColor(i));
			}
			show_mat(raw);
			
			int key = waitKey(1);
			if (key == 0x1B)
				break;
		}
	}

	return 0;
}
