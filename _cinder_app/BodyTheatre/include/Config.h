#pragma once

#include <string>

using std::string;

bool loadConfig(const char* config);
bool saveConfig(const char* config);

#define CONFIG_ITEM(type, var, default) extern type var;
#include "Config_def.h"
#undef CONFIG_ITEM

