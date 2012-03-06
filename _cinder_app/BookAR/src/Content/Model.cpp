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
			string value = it->getValue<string>();
			if (key == "animations")
			{
				BOOST_FOREACH(XmlTree item, it->getChildren())
				{
					// 					Animation* model = Animation::create(animation_item);
					// 					if (model)
					// 						mdl->models.push_back(shared_ptr<Model>(model));
				}
			}
			else if (key == "name")
				mdl->name = value;
			else if (key == "name_type")
				mdl->name_type = value;	
			else if (key == "on_click")
			{
				Function* fun = Function::create(value);
				if (fun)
					mdl->on_click = shared_ptr<Function>(fun); 
			}
			else if (key == "rotation_x")
				mdl->rotation_x = fromString<float>(value);
			else if (key == "rotation_y")
				mdl->rotation_y = fromString<float>(value);
			else if (key == "rotation_z")
				mdl->rotation_z = fromString<float>(value);
			else if (key == "scale")
				mdl->scale = fromString<float>(value);
			else if (key == "texture")
				mdl->texture_type = value;
			else if (key == "texture_type")
				mdl->texture_type = value;
			else if (key == "hidden")
				mdl->hidden = (value == "yes");
		}

		return mdl;
	}
}