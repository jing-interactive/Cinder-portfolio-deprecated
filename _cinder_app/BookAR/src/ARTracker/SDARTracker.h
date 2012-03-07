#ifndef SDAR_TRAKCER_H
#define SDAR_TRAKCER_H

#include "ARTracker.h"

class SDARTracker : public ARTracker
{
public:
	SDARTracker();
	~SDARTracker();
    void setup(int width, int height, double dNear, double dFar, void* param = NULL);

	//return num tracked
    unsigned int update(unsigned char* data);

	void getProjectionMat(ci::Matrix44d& mat);

	unsigned int getID(unsigned int tIdx);
	const char* getName(unsigned int tIdx);	
    void getModelViewMat(unsigned int tIdx, ci::Matrix44d& mat);
    void getCorners(unsigned int tIdx, ci::Vec2f points[4]);
private:
	int _width,_height;
	double _near, _far;
	unsigned int _n_trackables;
};

#endif //SDAR_TRAKCER_H