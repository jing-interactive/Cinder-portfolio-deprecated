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
			string value = it->getValue<string>();
			if (key == "name")
				scn->name = value;

			else if (key == "initialCommands")
				scn->initialCommands = value;
			else if (key == "models")
			{
				BOOST_FOREACH(XmlTree item, it->getChildren())
				{
					Model* model = Model::create(item);
					if (model)
					{
						model->parent = scn;
						scn->models.push_back(shared_ptr<Model>(model));
					}
				}
			}
		}

		return scn;
	}

	void Scene::draw()
	{
		BOOST_FOREACH(shared_ptr<Model> item, models)
		{
			item->draw();
		}
	}

	Scene::Scene()
	{
		parent = NULL;
	}

	void Scene::setMatrix( const ci::Matrix44d& modelview, const ci::Matrix44d& proj, const ci::Vec2f corners[4] )
	{
		this->modelview = modelview;
		this->proj = proj;
		for (int i=0;i<4;i++)
		{
			this->corners[i] = corners[i];
		}
	}
}