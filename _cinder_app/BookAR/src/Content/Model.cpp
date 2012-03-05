#include "Model.h"
#include <cinder/Xml.h>
#include <list>
#include "Model.h"
#include "Function.h"
#include <boost/foreach.hpp>

using namespace ci;
using namespace std;

namespace ARContent{

	Model* Model::create( const cinder::XmlTree& xmltree )
	{
		Model* mdl = new Model;
		const std::list<XmlTree>& properties= xmltree.getChildren();
		for (std::list<XmlTree>::const_iterator it = properties.begin();it != properties.end();++it)
		{
			string key = it->getValue<string>();
			it++;
			if (key == "name")
				mdl->name = key;
			else if (key == "texture")
				mdl->texture = key;	
			else if (key == "texture_type")
				mdl->texture_type = key;
			else if (key == "on_click")
			{
				Function* fun = Function::create(key);
				if (fun)
					mdl->on_click = shared_ptr<Function>(fun); 
			}
			else if (key == "animations")
			{
			//	BOOST_FOREACH(XmlTree item, it->getChildren())
				{
// 					Animation* model = Animation::create(animation_item);
// 					if (model)
// 						mdl->models.push_back(shared_ptr<Model>(model));
				}
			}
		}

		return mdl;
	}
}