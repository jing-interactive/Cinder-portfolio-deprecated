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
		image_double lsd_img;
		ntuple_list segs;//resulted line segments

		int W = input._size.width;
		int H = input._size.height;
		Mat frame;
		Mat gray(W, H, CV_8UC1);

		lsd_img = new_image_double(W,H);

		while (true)
		{
			Mat raw = input.get_frame(); 
			if (raw.empty())
				break;

			frame = raw.clone();

			if (input._InputType == input.From_Camera)
				flip(frame, frame, 1);

			cvtColor(frame, gray, CV_RGB2GRAY );
			//show_mat(gray);

			for(int y=0;y<H;y++)
				for(int x=0;x<W;x++)
				lsd_img->data[ x + y * W ] = gray.at<uchar>(y,x);
			segs = lsd(lsd_img);

#ifdef _DEBUG
			/* print output */ 		
			printf("%u line segments found:\n",segs->size);
#endif
			for(int i=0;i<segs->size;i++)
			{
				double* s = &segs->values[i*segs->dim];
				line(frame, Point(s[0],s[1]), Point(s[2],s[3]), vDefaultColor(i), s[4]);
			}

			show_mat(frame);
			int key = cvWaitKey(1);
			if (key == 0x1B)
				break;
		}

		free_image_double(lsd_img);
		free_ntuple_list(segs);
	}

	return 0;
}