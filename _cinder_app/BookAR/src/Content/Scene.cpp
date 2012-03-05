#include "Scene.h"
#include <cinder/Xml.h>
#include <list>
#include "Model.h"
#include <boost/foreach.hpp>

using namespace ci;
using namespace std;

namespace ARContent{

	Scene* Scene::create( const cinder::XmlTree& xmltree )
	{
		Scene* scn = new Scene;
		const std::list<XmlTree>& properties= xmltree.getChildren();
		for (std::list<XmlTree>::const_iterator it = properties.begin();it != properties.end();++it)
		{
			string key = it->getValue<string>();
			it++;
			if (key == "name")
				scn->name = key;//TODO:////fdffff

			else if (key == "initialCommands")
				scn->initialCommands = key;
			else if (key == "models")
			{
				BOOST_FOREACH(XmlTree item, it->getChildren())
				{
					Model* model = Model::create(item);
					if (model)
						scn->models.push_back(shared_ptr<Model>(model));
				}
			}
		}

		return scn;
	}

}