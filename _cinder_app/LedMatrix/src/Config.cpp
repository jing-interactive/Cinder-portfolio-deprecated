#include "Config.h"
#include <cinder/xml.h>
#include <cinder/app/App.h>
#include <cinder/Utilities.h>
#include <string>

using namespace ci;
using namespace ci::app;
using namespace std;

#define DO_ITEM(type, var, default) type var = default
#include "Config_def.h"
#undef DO_ITEM

#define FILE_TAG "LedMatrix"

void debug_puts( const char* str )
{
	if (DEBUG_MODE)
		puts(str);
}

bool loadConfig(const char* config)
{
	try
	{
		XmlTree tree(loadFile(config));
		XmlTree items = tree.getChild(FILE_TAG);

#define DO_ITEM(type, var, default) var = items.getChild(#var).getValue<type>()
#include "Config_def.h"
#undef DO_ITEM
	}
	catch( ... ) {
		printf("No file from from %s\n", config);
		return false;
	}
	printf("load from %s\n", config);
	return true;
}

bool saveConfig(const char* config)
{	
	try
	{
		XmlTree tree(FILE_TAG,"");

#define DO_ITEM(type, var, default) tree.push_back(XmlTree(#var, toString(var)))
#include "Config_def.h"
#undef DO_ITEM
		tree.write( writeFile(config));
	}
	catch( ... ) {
		console() << "[ERROR] Failed to save to " << config<<std::endl;
		return false;
	}
	return true;
}



