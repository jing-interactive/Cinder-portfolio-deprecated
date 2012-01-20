// Some of the source codes is extracted from eigenface.c, by Robin Hewitt, 2007

#include <algorithm>
#include <direct.h>

#include "../../_common/vOpenCV/OpenCV.h"
#include "../../_common/vOpenCV/BlobTracker.h"
#include "../../_common/CommonDialog.h" 
#include "recog.h"
#include "CvGabor.h"

using std::vector;
using std::string;

void get_time(IN char str[])
{
	time_t timep;
	tm *p;
	time(&timep);
	p = gmtime(&timep);

	sprintf(str, "%d-%d-%d__%d_%d_%d",
		1900+p->tm_year, 1+p->tm_mon, p->tm_mday,
		p->tm_hour, p->tm_min, p->tm_sec);
}

void show_help()
{
	printf("YourFace\n"
		"yet another face recognition software\n\n"
		"vinjn @ 2009\n\n"
		"使用说明\n"
		"+++++++++++++++++++++++++++++\n"
		"1. 先让程序找到你的脸\n"
		"2. 按【回车】保存你的脸，在media文件夹下新建一个属于你的文件夹\n"
		"3. 把你的脸都放在这里，多保存几张，识别率会高一些\n"
		"4. 按【ESC】关掉程序，再次打开，应该就能识别出你了\n"
		"+++++++++++++++++++++++++++++\n\n\n\n"
		);
}

char media_path[MAX_PATH];

void retrain(recognition_t& recog)
{
	recog.release();

	//[先遍历文件夹]
	char str_root[MAX_PATH];
	sprintf(str_root, "%s\\*", media_path);
	WIN32_FIND_DATA ffd;
	HANDLE hFind = FindFirstFile(str_root, &ffd);

	typedef char str_t[MAX_PATH];
	vector<string> dirs;
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			char* name = ffd.cFileName;
			if (strcmp(name, ".") && strcmp(name, ".."))
			{
				string a_dir(name); 
				dirs.push_back(a_dir);
			}
		}
	}
	while (FindNextFile(hFind, &ffd) != 0);

	//[遍历文件夹内的每个图片]
	int n = dirs.size();
	for (int i=0;i<n;i++)
	{
		const char* a_dir = dirs[i].c_str();
		char adir_root[MAX_PATH];
		sprintf(adir_root, "%s\\*", a_dir);
		hFind = FindFirstFile(adir_root, &ffd);
		do
		{
			if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			{
				char* name = ffd.cFileName;
				char full[MAX_PATH];//全路径
				sprintf(full, "%s\\%s", a_dir, name);

				IplImage* img = cvLoadImage(full, CV_LOAD_IMAGE_GRAYSCALE);
				if (img)
				{
					recog.add_profile(img, a_dir);//在此进行添加
				}
			}
		}
 		while (FindNextFile(hFind, &ffd) != 0);
	}

	//[最后 学习]
	recog.learn();

}

int CALLBACK BrowseCallbackProc(HWND hwnd,UINT uMsg,LPARAM lParam,LPARAM lpData)
{
    if(uMsg == BFFM_INITIALIZED)
    {
        SendMessage(hwnd, BFFM_SETSELECTION,
            TRUE,(LPARAM)(media_path));
    }
    return 0;
}


int main(int argc, char** argv )
{
	_chdir("../media");
	GetCurrentDirectory(MAX_PATH, media_path);	
	
	char filename[MAX_PATH]="\0";
	char szFilter[]= "Image (*.jpg)\0*.jpg\0";
	//[dlg_init]
	HWND hWnd;
		char title[512];
		GetConsoleTitle(title, sizeof(title));
		hWnd = FindWindow(NULL, title);
	CComDialog save_dlg(hWnd);
	save_dlg._ofn.lpstrInitialDir = media_path;		

	VideoInput video_input;

	if (video_input.init(argc,argv))
	{
		show_help();		

		vHaarFinder haar;
		if (!haar.init("haarcascade_frontalface_alt.xml"))
		{
			printf("Failed to load haarcascade_frontalface_alt.xml\n");
			goto _error;
		}

		recognition_t recog;

		retrain(recog);

		bool exit = false;
		vector<CvRect> regions;

		IplImage* frame = cvCreateImage(video_input._half_size, 8, 3);
		IplImage* grey = cvCreateImage(video_input._half_size, 8, 1);

		vector<IplImage*> face_imgs;
		face_imgs.resize(8);

		for (int i=0;i<face_imgs.size();i++)
			face_imgs[i] = cvCreateImage(cvSize(92,112), 8, 1);
		
		while (!exit)
		{
			IplImage* raw = video_input.get_frame();
			cvFlip(raw, raw, 1);

			cvResize(raw, frame);
			vGrayScale(frame, grey);
			haar.find(frame, 1, false);

			int n_blobs = std::min<int>(haar.blobs.size(), 1);
			for (int i=0;i<n_blobs;i++)
			{
				vBlob& obj = haar.blobs[i];
				cvSetImageROI(grey, obj.box);
				cvResize(grey, face_imgs[i]);
				cvResetImageROI(grey);
				char* found_name = recog.recognize(face_imgs[i]);
				//putText(raw, region.x, region.y, found_name, CV_RGB(0,0,0));
				vDrawText(frame, obj.center.x, obj.center.y,found_name,  CV_RGB(50,100,255));
				show_image(face_imgs[i]);
				//char name[256];
				//sprintf(name, "face %d", i);
				//cvNamedWindow(found_name);
				//cvShowImage(name, face_imgs[i]); 
			}

			show_image(frame);

			int key = cvWaitKey(1);
			switch (key)
			{
			case VK_ESCAPE: 
				exit = true;
				break;
			case VK_RETURN:
				//if (save_dlg.PopFileSaveDlg(filename, szFilter, "jpg"))
#if 0				
				{
					printf("请输入你的名字:   ");
					scanf("%s", filename);

					//[创建文件夹]
					char command[MAX_PATH];
					sprintf(command, "mkdir %s", filename);
					system(command);

					//[保存文件]
					char c_time[MAX_PATH];
					get_time(c_time);
					sprintf(filename, "%s\\%s.jpg", filename, c_time);
					cvSaveImage(filename, face_imgs[0]);
					printf("图片保存为 %s\n", filename);
				}
#else
				if (save_dlg.PopFileSaveDlg(filename, szFilter, "jpg"))
				{
					////[创建文件夹]
					//char command[MAX_PATH];
					//sprintf(command, "mkdir %s", filename);
					//system(command);

					////[保存文件]
					//char c_time[MAX_PATH];
					//get_time(c_time);
					//sprintf(filename, "%s\\%s.jpg", filename, c_time);
					cvSaveImage(filename, face_imgs[0]);
					printf("图片保存为 %s\n", filename);
				}
#endif
				break;
#ifdef _DEBUG
			case 'r':
				retrain(recog);
				//cvReleaseImage(&grey);
				//grey = cvCreateImage(cvGetSize(raw), 8, 1);
				printf("人脸库已重新训练过\n");
				break;
#endif
			default:
				break;
			}
		}
	}

	return 0;

_error:
	system("pause");
	return 0;
} 