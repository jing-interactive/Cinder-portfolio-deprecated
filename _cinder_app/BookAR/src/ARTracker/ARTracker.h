#ifndef AR_TRAKCER_H
#define AR_TRAKCER_H

#include <cinder/Matrix.h>
#include <string>

class ARTracker
{
public:
	static ARTracker* create(const std::string& type);
    virtual ~ARTracker(){}

    virtual void setup(int width, int height, double dNear, double dFar, void* param = NULL) = 0;

    virtual unsigned int update(unsigned char* data) = 0;

	virtual void getProjectionMat(ci::Matrix44d& mat) = 0;
	virtual unsigned int getID(unsigned int tIdx) = 0;
	virtual const char* getName(unsigned int tIdx) = 0;
    virtual void getModelViewMat(unsigned int tIdx, ci::Matrix44d& mat) = 0;
    virtual void getCorners(unsigned int tIdx, ci::Vec2f points[4]) = 0;
};

#endif //AR_TRAKCER_H