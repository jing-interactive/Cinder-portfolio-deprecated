#include "SDARTracker.h"
#include "SDAR.h"
#include <vector>
#include "cinder/Matrix.h"
#include "cinder/app/App.h"

using std::string;
using std::vector;

void SDARTracker::setup( int width, int height,double dNear, double dFar, void* param/* = NULL*/)
{
	_width = width;
	_height = height;
	SDARStart(width, height);
	const vector<string>& mdl_files = *(vector<string>*)(param);
	SDARStart(width, height);
	int bRet = 0;
	for (int i=0;i<mdl_files.size();i++)
	{
		bRet = SDARLoad((char*)(ci::app::getAppPath().generic_string()+mdl_files[i]).c_str());
		if(!bRet)
		{

		}
	}
	_near = dNear;
	_far = dFar;
}

SDARTracker::SDARTracker()
{

}

SDARTracker::~SDARTracker()
{
	SDAREnd();
}

void SDARTracker::update( unsigned char* data )
{
	SDARTrack(data, _width*3);
}

unsigned int SDARTracker::getNumTracked()
{
	return getNumOfActiveTrackables();
}

void SDARTracker::getProjectionMat(ci::Matrix44d& mat)
{
	double* m = getProjectionMatrix(_near, _far);
	mat = ci::Matrix44d(m);
}

void SDARTracker::getModelViewMat(unsigned int tIdx, ci::Matrix44d& mat)
{
	double* m = getModelViewMatrix(tIdx);
	mat = ci::Matrix44d(m);
}

void SDARTracker::getCorners(unsigned int tIdx, ci::Vec2f points[4])
{
	for(int i=0; i<4; i++)
	{
		points[i].x = getVertexX(tIdx, i);
		points[i].y = getVertexY(tIdx, i);
	}
}

unsigned int SDARTracker::getID( unsigned int tIdx )
{
	return getActiveTrackableID(tIdx);
}

