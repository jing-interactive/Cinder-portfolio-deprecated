#pragma once

#include <string>
using std::string;

#define CONFIG_FILE_NAME "MiniConfig.xml"

#define ITEM_DEF(type, var, default) extern type var;
#include "item.def"
#undef ITEM_DEF

// item.def is the place to define your global variables
// in the format of
//
// 
// ITEM_DEF(int, BG_W, 882)
// ITEM_DEF(int, BG_H, 725)
// ITEM_DEF(string, NAME, "vinjn")
// ITEM_DEF(float, velocity, 3.0f)

void readConfig();
void writeConfig();
