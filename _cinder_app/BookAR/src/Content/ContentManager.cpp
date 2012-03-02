#include "ContentManager.h"
#include "cinder/Xml.h"
#include "cinder/app/App.h"
#include "Content.h"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace ARContent{

bool ContentManager::load( const std::string& plist )
{
	try
	{
		XmlTree doc(loadFile(plist));
		XmlTree firstContent = doc.getChild("plist/dict");

		for( XmlTree::Iter item = firstContent.begin(); item != firstContent.end(); ++item )
		{
			//key
			string mdl_key = item->getValue<string>();
			item++;
			Content* ctt = Content::create(*item);
			if (ctt)
			{
				_contents.insert(Pair(mdl_key, shared_ptr<Content>(ctt)));
			}
		}

	}
	catch( ... ) {
		console() << "[ERROR] Failed to load ContentManager from " << plist<<std::endl;
		return false;
	}

	return true;
}

}
