#include "Content.h"
#include <cinder/Xml.h>
#include <cinder/app/App.h>
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
			string value = it->getValue<string>();
			if (key == "name")	ctt->name = value;
			else if (key == "type")	ctt->type = value;
			else if (key == "sync")	ctt->sync = value;	
			else if (key == "scenes")
			{
				BOOST_FOREACH(XmlTree item, it->getChildren())
				{
					Scene* scene = Scene::create(item);
					if (scene)
					{
						scene->parent = ctt;
						ctt->scenes.push_back(shared_ptr<Scene>(scene));
					}
				}
			}
		}
		if (!ctt->scenes.empty())
			ctt->_current_scene = ctt->scenes.front();

		return ctt;
	}

	void Content::draw()
	{
		if (_current_scene)
			_current_scene->draw();
	}

	std::shared_ptr<class Scene> Content::getCurrentScene()
	{
		return _current_scene;
	}

	void Content::setCurrentSceneByName( const std::string& name )
	{
		shared_ptr<Scene> scn;
		BOOST_FOREACH(shared_ptr<Scene> item, scenes)
		{
			if (item->name == name)
			{
				scn = item;
				break;
			}
		}
		if (scn)
			_current_scene = scn;
	}
}