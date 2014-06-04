#pragma once

#include <iostream>
#include "Cinder/Color.h"

using namespace std;
using namespace ci;

// TODO: proto-buf?
struct AnimConfig
{
    static const int kCount = 11; // anim: [0, 9), kinect: 10
    static const int kKinect = kCount - 1;

    AnimConfig();

    Color getColor() const;

    float lightValue;
    float lightValue2; // if non-zero, then random light value from (lightValue, lightValue2)
    // see getColor()
    int loopCount; // bigger than 1, or zero means don't play
};

ostream& operator<<(ostream& lhs, const AnimConfig& rhs);
istream& operator>>(istream& lhs, AnimConfig& rhs);

struct Config
{
    static const int kCount = 6;

    AnimConfig animConfigs[AnimConfig::kCount];
};

ostream& operator<<(ostream& lhs, const Config& rhs);
istream& operator>>(istream& lhs, Config& rhs);

const int kHourCount = 24; // valid hours for G9 are [10~23; 00; 01]

extern Config mConfigs[Config::kCount];
extern int mConfigIds[kHourCount];

void readProgramSettings();
void writeProgramSettings();

