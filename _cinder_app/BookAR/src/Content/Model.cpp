#include "Model.h"
#include <list>
#include "Model.h"
#include "Function.h"
#include "Animation.h"
#include "Scene.h"
#include <boost/foreach.hpp>
#include <cinder/app/AppBasic.h>
#include <cinder/ImageIo.h>
#include <cinder/Xml.h>

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
				{
					fun->parent = mdl;
					mdl->on_click = shared_ptr<Function>(fun);
				}
			}
			else if (key == "translation_x")
				mdl->translation.x = fromString<float>(value);
			else if (key == "translation_y")
				mdl->translation.y = fromString<float>(value);
			else if (key == "translation_z")
				mdl->translation.z = fromString<float>(value);
			else if (key == "rotation_x")
				mdl->rotation.x = fromString<float>(value);
			else if (key == "rotation_y")
				mdl->rotation.y = fromString<float>(value);
			else if (key == "rotation_z")
				mdl->rotation.z = fromString<float>(value);
			else if (key == "scale")
				mdl->scale = fromString<float>(value);
			else if (key == "texture")
				mdl->texture = value;
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
		if (!gl_texture)//delayed creating texture
			gl_texture = loadImage(getAppPath()/texture);
		BOOST_FOREACH(shared_ptr<Animation> item, animations)
		{
		//	item->draw();
		}

		if (name == "plane")
		{//special plane
			gl::disableDepthWrite();

			gl_texture.enableAndBind();			
			//			gl::pushMatrices();			
			{
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				gl::translate(translation);
				gl::scale(scale,scale);

				glBegin(GL_QUADS);
				gl::color(1,1,1);
				glTexCoord2f(0.0f, 0.0f);
				glVertex3f(parent->corners[0].x,parent->corners[0].y,0.5);
				glTexCoord2f(1.0f, 0.0f);
				glVertex3f(parent->corners[1].x,parent->corners[1].y,0.5);
				glTexCoord2f(1.0f, 1.0f);
				glVertex3f(parent->corners[2].x,parent->corners[2].y,0.5);
				glTexCoord2f(0.0f, 1.0f);
				glVertex3f(parent->corners[3].x,parent->corners[3].y,0.5);
				glEnd();
			}
			//			gl::popMatrices();
			gl_texture.disable();
		}
	}

	bool Model::mouseUp( ci::app::MouseEvent event )
	{
		if (on_click)
			on_click->execute();
		BOOST_FOREACH(std::shared_ptr<class Function> fun, click_actions)
			fun->execute();

		return false;
	}

	Model::Model()
	{
		parent = NULL;
	}

}