#include <cinder/xml.h>
#include <cinder/app/App.h>
#include <cinder/Utilities.h>
#include <string>
#include "Config.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define CONFIG_ITEM(type, var, default) type var = default;
#include "Config_def.h"
#undef CONFIG_ITEM

#define FILE_TAG "CinderApp"

bool loadConfig(const char* config)
{
	try
	{
		XmlTree tree(loadFile(config));
		XmlTree items = tree.getChild(FILE_TAG);

#define CONFIG_ITEM(type, var, default) var = items.getChild(#var).getValue<type>();
#include "Config_def.h"
#undef CONFIG_ITEM
	}
	catch( ... ) {
		printf("Error happens while loading %s\n", config);
		return false;
	}
	printf("Loaded from %s\n", config);
	return true;
}

bool saveConfig(const char* config)
{	
	try
	{
		XmlTree tree(FILE_TAG,"");

#define CONFIG_ITEM(type, var, default) tree.push_back(XmlTree(#var, toString(var)));
#include "Config_def.h"
#undef CONFIG_ITEM
		tree.write( writeFile(config));
	}
	catch( ... ) {
		printf("Error happens while saving %s\n", config);
		return false;
	}
	printf("Saved to %s\n", config);
	return true;
}