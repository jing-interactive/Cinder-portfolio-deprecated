#include "../_common/vOpenCV/OpenCV.h"
extern "C"{
#include "../_common/lsd-1.5/lsd.h"
}

using namespace cv;

VideoInput input;

int main(int argc, char** argv )
{	
	if (input.init(argc,argv))
	{
		image_double lsd_in;
		ntuple_list lsd_out;

		int W = input._size.width;
		int H = input._size.height;
		Mat gray(W, H, CV_8UC1);

		lsd_in = new_image_double(W,H);

		while (true)
		{
			Mat raw = input.get_frame(); 
			if (raw.empty())
				break;

			cvtColor(raw, gray, CV_RGB2GRAY );
			show_mat(gray);

			for(int y=0;y<H;y++)
				for(int x=0;x<W;x++)
				lsd_in->data[ x + y * W ] = gray.at<uchar>(y,x);
			lsd_out = lsd(lsd_in);

			/* print output */ 		
			printf("%u line segments found:\n",lsd_out->size);
			for(int i=0;i<lsd_out->size;i++)
				for(int j=0;j<lsd_out->dim;j++)
					printf("%f ",lsd_out->values[ i * lsd_out->dim + j ]);

			show_mat(raw);
			int key = cvWaitKey(1);
			if (key == 0x1B)
				break;
		}

		free_image_double(lsd_in);
		free_ntuple_list(lsd_out);
	}

	return 0;
}