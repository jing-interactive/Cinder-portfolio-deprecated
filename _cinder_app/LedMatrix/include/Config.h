#pragma once

bool loadConfig(const char* config);
bool saveConfig(const char* config);

void debug_puts(const char* str);

#define DO_ITEM(type, var, default) extern type var;
#include "Config_def.h"
#undef DO_ITEM

