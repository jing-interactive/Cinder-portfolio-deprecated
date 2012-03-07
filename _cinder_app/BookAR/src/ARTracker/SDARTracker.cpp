#include "SDARTracker.h"
#include "SDAR.h"
#include <vector>
#include <cinder/Matrix.h>
#include <cinder/app/App.h>

using std::string;
using std::vector;

void SDARTracker::setup( int width, int height,double dNear, double dFar, void* param/* = NULL*/)
{
	_width = width;
	_height = height;
	SDARStart(width, height);
	const vector<string>& mdl_files = *(vector<string>*)(param);
	SDARStart(width, height);
	_n_trackables = 0;
	for (int i=0;i<mdl_files.size();i++)
	{
		if(SDARLoad((char*)(ci::app::getAppPath().generic_string()+mdl_files[i]).c_str()))
		{
			
		}
	}
	_n_trackables = getNumOfTrackables();
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

unsigned int SDARTracker::update( unsigned char* data )
{
	SDARTrack(data, _width*3);
	return getNumOfActiveTrackables();
}

void SDARTracker::getProjectionMat(ci::Matrix44d& mat)
{
	double* m = getProjectionMatrix(_near, _far);
	mat = ci::Matrix44d(m);
}

void SDARTracker::getModelViewMat(unsigned int tIdx, ci::Matrix44d& mat)
{
	assert(tIdx >=0 && tIdx < _n_trackables);
	double* m = getModelViewMatrix(tIdx);
	mat = ci::Matrix44d(m);
}

void SDARTracker::getCorners(unsigned int tIdx, ci::Vec2f points[4])
{
	assert(tIdx >=0 && tIdx < _n_trackables);
	for(int i=0; i<4; i++)
	{
		points[i].x = getVertexX(tIdx, i);
		points[i].y = getVertexY(tIdx, i);
	}
}

unsigned int SDARTracker::getID( unsigned int tIdx )
{
	assert(tIdx >=0 && tIdx < _n_trackables);
	return getActiveTrackableID(tIdx);
}

const char* SDARTracker::getName( unsigned int tIdx )
{
	assert(tIdx >=0 && tIdx < _n_trackables);
	return getActiveTrackableName(tIdx);
}

