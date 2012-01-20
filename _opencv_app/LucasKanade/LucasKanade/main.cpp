#include "../../../_common/vOpenCV/OpenCV.h"
#include "../../../_common/MiniTimer.h"

VideoInput input;

using namespace cv;

const int MAX_COUNT = 500;

typedef std::deque<Point2f> one_track;

int main(int argc, char** argv )
{
	int track_len = 10;
	int detect_interval = 5;
	//	self.tracks = [];
	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,10,0.03);
	Size subPixWinSize(10,10), winSize(15,15);

	if (input.init(argc,argv))
	{
		Mat frame, prev_gray, frame_gray,mask;
		std::vector<one_track> tracks;

		while (true)
		{
			MiniTimer tm;

			IplImage* _raw = input.get_frame(); 
			if (!_raw)
				break;
			Mat raw = _raw;

			flip(raw, frame, 1);
			cvtColor(frame, frame_gray, CV_BGR2GRAY);
			if (mask.empty())
				mask = Mat(frame_gray.size(), frame_gray.type(), CV_RGB(255,255,255));

			tm.resetStartTime();
			if (tracks.size() > 0)
			{  
				vector<uchar> status;
				vector<float> err;

				std::vector<Point2f> p0,p1,p0r;
				for (int i=0;i<tracks.size();i++)
				{
					const one_track& tr = tracks[i]; 
					p0.push_back(tr.back());
				}
				calcOpticalFlowPyrLK(prev_gray, frame_gray, p0, p1, 
					status, err, winSize, 3, termcrit, 0, 0, 0.001);
				calcOpticalFlowPyrLK(frame_gray, prev_gray, p1, p0r,
					status, err, winSize, 3, termcrit, 0, 0, 0.001);
				std::vector<Point2f> d;
				subtract(p0,p0r,d);
				std::vector<one_track> new_tracks;
				for (int i=0;i<tracks.size();i++)
				{
					one_track& tr = tracks[i];
					if (norm(d[i]) < 1)//good enough
					{
						tr.push_back(p1[i]);
						if (tr.size() > track_len)
							tr.pop_front();
						new_tracks.push_back(tr);
					}
				}
				tracks = new_tracks;
				for (int i=0;i<tracks.size();i++)
				{
					one_track& tr = tracks[i];
					for (int k=0;k<tr.size()-1;k++)
					{
						line(frame, tr[k], tr[k+1], CV_RGB(0, 255, 0));
					}
				}
				char info[256];
				sprintf(info, "track count: %d", tracks.size());
				IplImage ipl = frame;
				vDrawText(&ipl, 20, 20, info);
			}
			tm.profileFunction("calcOpticalFlowPyrLK");

			if (input._frame_num % detect_interval == 0)
			{
				tm.resetStartTime();
				for (int i=0;i<tracks.size();i++)
				{
					one_track& tr = tracks[i];
					circle(mask, tr.back(), 5, CV_RGB(0,0,0), CV_FILLED);
				}

				std::vector<Point2f> corners;
				goodFeaturesToTrack(frame_gray, corners, 500, 0.1, 7,mask,7);
				for (int i=0;i<corners.size();i++)
				{
					one_track tr;
					tr.push_back(corners[i]);
					tracks.push_back(tr);
				}
				tm.profileFunction("goodFeaturesToTrack");
			}

			prev_gray = frame_gray.clone();
			show_mat(frame);

			int key = waitKey(1);
			if (key == 0x1B)
				break;
		}
	}

	return 0;
}
