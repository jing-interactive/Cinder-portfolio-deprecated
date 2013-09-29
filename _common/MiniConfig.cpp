#include "MiniConfig.h"
#include <cinder/xml.h>
#include <cinder/Utilities.h>
#include <cinder/app/App.h>
#include <cinder/params/Params.h>

using namespace ci;
using namespace ci::app;
using namespace std;

#define GROUP_DEF(grp)
#define ITEM_DEF(type, var, default) type var = default;
#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF

namespace
{
    const std::string FILE_TAG = "root";

    void revertToDefaultValues()
    {
#define GROUP_DEF(grp)
#define ITEM_DEF(type, var, default) var = default;
#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF
        console() << "MiniConfig reverted to default values" << endl;
    }
}

void readConfig()
{
	fs::path configPath = getAssetPath("./")/CONFIG_FILE_NAME;
	try
	{
		XmlTree tree(loadFile(configPath));
		XmlTree root = tree.getChild(FILE_TAG);
        XmlTree group;

#define GROUP_DEF(grp) group = root.getChild(#grp);
#define ITEM_DEF(type, var, default) \
    do \
    {\
        if (group.getTag().empty())\
            var = root.getChild(#var).getValue<type>();\
        else\
            var = group.getChild(#var).getValue<type>();\
    } while (0);
#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF
        console() << "Reads from " << configPath.string() << endl;
	}
	catch( ... ) {
		console() << "[Warning] Fails to read from " << configPath.string()<<endl;
        revertToDefaultValues();
		writeConfig();
	}
}

void writeConfig()
{
	fs::path configPath = getAssetPath("./")/CONFIG_FILE_NAME;
	try
	{
		XmlTree tree(FILE_TAG, "");
        XmlTree group;

#define GROUP_DEF(grp) \
        do \
        {\
            if (!group.getTag().empty()) tree.push_back(group);\
            group = XmlTree(#grp, "");\
        } while (0);

#define ITEM_DEF(type, var, default) \
        do \
        {\
            XmlTree item(#var, toString(var));\
            if (group.getTag().empty()) tree.push_back(item);\
            else group.push_back(item);\
        } while (0);

#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF
        if (!group.getTag().empty()) tree.push_back(group);

#ifdef CHINESE_GBK_ENCODING_ENABLED
        OStreamRef os = writeFile(configPath)->getStream();
        std::ofstream of(configPath.string().c_str());
        const std::string kGbkHeader = "<?xml version=\"1.0\" encoding=\"gbk\"?>";
        of << kGbkHeader << std::endl << tree;
#else
        tree.write( writeFile(configPath));
#endif
		console() << "Writes to " << configPath.string() <<endl;
	}
	catch( ... ) {
		console() << "[Warning] Fails to write to " << configPath.string() <<endl;
	}
}

void setupConfigUI(cinder::params::InterfaceGl* params)
{
#define GROUP_DEF(grp)                  params->addSeparator();       
#define ITEM_DEF(type, var, default)    params->addParam(#var, &var);
#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF
    params->addSeparator();
    params->addButton("SAVE", writeConfig);
}

int getConfigUIHeight()
{
    const int kItemHeight = 20;
    int height = kItemHeight * 2; // top + bottom

#define GROUP_DEF(grp)                  height += kItemHeight;
#define ITEM_DEF(type, var, default)    height += kItemHeight;
#include "item.def"
#undef ITEM_DEF
#undef GROUP_DEF
    return height;
}
