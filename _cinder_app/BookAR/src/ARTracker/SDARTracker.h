#ifndef SDAR_TRAKCER_H
#define SDAR_TRAKCER_H

#include "ARTracker.h"

class SDARTracker : public ARTracker
{
public:
	SDARTracker();
	~SDARTracker();
    void setup(int width, int height, double dNear, double dFar, void* param = NULL);
    void update(unsigned char* data);
    unsigned int getNumTracked();

	unsigned int getID(unsigned int tIdx);
	void getProjectionMat(ci::Matrix44d& mat);
    void getModelViewMat(unsigned int tIdx, ci::Matrix44d& mat);
    void getCorners(unsigned int tIdx, ci::Vec2f points[4]);
private:
	int _width,_height;
	double _near, _far;
};

#endif //SDAR_TRAKCER_H