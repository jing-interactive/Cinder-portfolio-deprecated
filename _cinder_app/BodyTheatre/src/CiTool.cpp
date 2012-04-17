#include "CiTool.h"
#include "cinder/Area.h"
#include "cinder/gl/gl.h"
#include "boost/foreach.hpp"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "cinder/app/App.h"
#include "CinderOpenCV.h"
#include "cinder/ip/Resize.h"
#include <shellapi.h>

namespace cinder{

	Vec3f screenToWorld( const Vec2i &mouse, float z )
	{
		Vec3f p = Vec3f(mouse, z);

		/* adjust y (0,0 is lowerleft corner in OpenGL) */ 
		Area viewport = gl::getViewport();
		p.y = (viewport.getHeight() - p.y);

		/* near plane intersection */
		p.z = 0.0f;
		Vec3f p0 = unproject(p);

		/* far plane intersection */
		p.z = 1.0f;
		Vec3f p1 = unproject(p);

		/* find (x, y) coordinates */
		float t = (z - p0.z) / (p1.z - p0.z);
		p.x = (p0.x + t * (p1.x - p0.x));
		p.y = (p0.y + t * (p1.y - p0.y));
		p.z = z;

		return p;
	}

	Vec3f unproject( const Vec3f &pt )
	{
		/* find the inverse modelview-projection-matrix */
		Matrix44f a = gl::getProjection() * gl::getModelView();
		a.invert();

		/* transform to normalized coordinates in the range [-1, 1] */
		Area viewport = gl::getViewport();
		Vec4f in;
		in.x = (pt.x - viewport.getX1())/viewport.getWidth()*2.0f-1.0f;
		in.y = (pt.y - viewport.getY1())/viewport.getHeight()*2.0f-1.0f;
		in.z = 2.0f * pt.z - 1.0f;
		in.w = 1.0f;

		/* find the object's coordinates */
		Vec4f out = a * in;
		if(out.w != 0.0f) out.w = 1.0f / out.w;

		/* calculate output */
		Vec3f result;
		result.x = out.x * out.w;
		result.y = out.y * out.w;
		result.z = out.z * out.w;

		return result;
	}

	void writeImages( std::vector<Surface8u>& images, fs::path& familyName)
	{
		app::console() << familyName << std::endl;
		int i = 0;
		char name[100];
		Surface8u small(320, 240, false);
		BOOST_FOREACH(Surface8u& img, images)
		{
			sprintf(name, "%02d.jpg", i++);
			ip::resize<uint8_t>(img, &small);
/*			cv::Mat frame = toOcv(img);*/
			writeImage(familyName/std::string(name), small);
		}
	}

	void postImageToWeibo( std::string& fileName, std::string& text )
	{

	}

	void execute( string exeFile, string param )
	{
		char command[256];
		sprintf(command, "%s %s", exeFile.c_str(), param.c_str());

		STARTUPINFOA startupinfo;
		GetStartupInfoA (&startupinfo);

		PROCESS_INFORMATION pro2info; 

		CreateProcessA(exeFile.c_str(), (LPSTR)param.c_str(), NULL, NULL, false, CREATE_DEFAULT_ERROR_MODE, NULL,
			NULL, &startupinfo, &pro2info);

		WaitForSingleObject (pro2info.hProcess, INFINITE);


//		::ShellExecuteA( NULL, "open", exeFile.c_str(), NULL, NULL, SW_SHOWNORMAL );
// 		char command[256];
// 		sprintf(command, "%s %s", exeFile.c_str(), param.c_str());
// 		::system(command);
	}
}