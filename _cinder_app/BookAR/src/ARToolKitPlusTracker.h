#ifndef ARTOOLKITPLUS_TRAKCER_H
#define ARTOOLKITPLUS_TRAKCER_H

#include "cinder/Matrix44.h"

class ARTracker
{
    void setup(int width, int height) = 0;
    void update(unsigned char* data) = 0;
    unsigned int getNumOfTrackables() = 0;
    Matrix44d getModelViewMatrix(unsigned int tIdx) = 0;
    Matrix44d getProjectionMatrix() = 0;
    ci::Vec2f[4] getCorners(unsigned int tIdx) = 0;
};

#endif //ARTOOLKITPLUS_TRAKCER_H