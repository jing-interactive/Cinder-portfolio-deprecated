#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#pragma once
#include <string>
#include <iostream>
#include <fstream>

#ifdef _UNICODE
typedef std::wstring tstring;
typedef std::wifstream tifstream;
typedef std::wofstream tofstream;
#else
typedef std::string tstring;
typedef std::ifstream tifstream;
typedef std::ofstream tofstream;
#endif

#endif//__CONFIGURATION_H__