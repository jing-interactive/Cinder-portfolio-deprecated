#pragma once

#include <string>
using std::string;

#ifdef RICH_CINDER_FORMAT_ENABLED
    #include "cinder/Vector.h"
    #include "cinder/Quaternion.h"
    #include "cinder/Color.h"

    using ci::Vec3f;
    using ci::Quatf;
    using ci::Color;
    using ci::ColorA;
#endif

#define CONFIG_FILE_NAME "MiniConfig.xml"

#define GROUP_DEF(grp)
#define ITEM_DEF(type, var, default) extern type var;
#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF

// item.def is the place to define your global variables
// in the format of
//
// 
// ITEM_DEF(int, APP_WIDTH, 882)
// ITEM_DEF(int, APP_HEIGHT, 725)
// ITEM_DEF(string, NAME, "vinjn")
// ITEM_DEF(float, velocity, 3.0f)

void readConfig();
void writeConfig();

namespace cinder { namespace params {
class InterfaceGl;
} }

void setupConfigUI(cinder::params::InterfaceGl* params);
int getConfigUIHeight();