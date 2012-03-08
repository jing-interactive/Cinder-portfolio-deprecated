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

	ci::Matrix44d getProjectionMat();

	unsigned int getID(unsigned int tIdx);
	const char* getName(unsigned int tIdx);	
    ci::Matrix44d getModelViewMat(unsigned int tIdx);
    void getCorners(unsigned int tIdx, ci::Vec2f points[4]);
private:
	int _width,_height;
	double _near, _far;
	unsigned int _n_trackables;
};

#endif //SDAR_TRAKCER_H