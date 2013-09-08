#include "MiniConfig.h"
#include <cinder/xml.h>
#include <cinder/Utilities.h>
#include <cinder/app/App.h>

using namespace ci;
using namespace ci::app;
using namespace std;

#ifndef CONFIG_FILE_NAME
#define CONFIG_FILE_NAME "MiniConfig.xml"
#endif

#define ITEM_DEF(type, var, default) type var = default;
#include "item.def"
#undef ITEM_DEF

const std::string FILE_TAG = "root";

void readConfig()
{
	fs::path configPath = getAssetPath("./")/CONFIG_FILE_NAME;
	try
	{
		XmlTree tree(loadFile(configPath));
		XmlTree items = tree.getChild(FILE_TAG);
#define ITEM_DEF(type, var, default) var = items.getChild(#var).getValue<type>();
#include "item.def"
#undef ITEM_DEF
		console() << "Reads from " << configPath.string() << endl;
	}
	catch( ... ) {
		console() << "[Warning] Fails to read from " << configPath.string()<<endl;
		writeConfig();
	}
}

void writeConfig()
{
	fs::path configPath = getAssetPath("./")/CONFIG_FILE_NAME;
	try
	{
		XmlTree tree(FILE_TAG,"");
#define ITEM_DEF(type, var, default) tree.push_back(XmlTree(#var, toString(var)));
#include "item.def"
#undef ITEM_DEF
		tree.write( writeFile(configPath));
		console() << "Writes to " << configPath.string() <<endl;
	}
	catch( ... ) {
		console() << "[Warning] Fails to write to " << configPath.string() <<endl;
	}
}
