#include "Model.h"
#include <cinder/Xml.h>
#include <list>
#include "Model.h"
#include "Function.h"
#include "Animation.h"
#include <boost/foreach.hpp>
#include <cinder/app/AppBasic.h>

using namespace ci;
using namespace ci::app;
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

//		App::get()->registerMouseUp(mdl, &(Model::mouseUp));

		return mdl;
	}

	void Model::draw()
	{
// 		BOOST_FOREACH(shared_ptr<Animation> item, animations)
// 		{
// 			item->draw();
// 		}
	}

	bool Model::mouseUp( ci::app::MouseEvent event )
	{
		if (on_click)
			on_click->execute();
		BOOST_FOREACH(std::shared_ptr<class Function> fun, click_actions)
			fun->execute();

		return false;
	}

}