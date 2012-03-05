#include "Content.h"
#include <cinder/Xml.h>
#include "cinder/app/App.h"
#include <boost/foreach.hpp>
#include "Scene.h"

using namespace std;
using namespace ci;
using namespace ci::app;

namespace ARContent{

	Content* Content::create( const cinder::XmlTree& xmltree )
	{
		Content* ctt = new Content;
		const std::list<XmlTree>& properties = xmltree.getChildren();
		for (std::list<XmlTree>::const_iterator it = properties.begin();it != properties.end();++it)
		//BOOST_FOREACH(XmlTree dict_items, xmltree.getChildren())
		{
			string key = it->getValue<string>();
			it++;
			if (key == "name")	ctt->name = it->getValue<string>();
			else if (key == "type")	ctt->type = it->getValue<string>();
			else if (key == "sync")	ctt->sync = it->getValue<string>();	
			else if (key == "scenes")
			{
				BOOST_FOREACH(XmlTree item, it->getChildren())
				{
					Scene* scene = Scene::create(item);
					if (scene)
						ctt->scenes.push_back(shared_ptr<Scene>(scene));
				}
			}
		}

		return ctt;
	}
}