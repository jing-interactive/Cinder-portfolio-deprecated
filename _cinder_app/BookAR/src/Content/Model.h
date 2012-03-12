#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <string>
#include <cinder/Cinder.h>
#include <cinder/app/MouseEvent.h>
#include <cinder/gl/Texture.h>
//#include <cinder/Timeline.h>

namespace cinder{
	class XmlTree;
}

namespace ARContent{

class Model
{
public:
	static Model* create(const cinder::XmlTree& xmltree);
	//OPTIONAL
	std::vector<std::shared_ptr<class Animation>> animations;

	void draw();
	bool mouseUp(ci::app::MouseEvent event);

	std::string name;
	std::string name_type;//always "MD2"

	//gotolink::::http://www.youtube.com/watch?v=TWSDaF73sOc
	//TODO: on_click and click_actions won't exist together, they might be combined
	std::shared_ptr<class Function> on_click;
	std::vector<std::shared_ptr<class Function>> click_actions;

	ci::Vec3f translation;
	ci::Vec3f rotation;
	float scale;

	std::string texture;
	std::string texture_type;

	std::shared_ptr<class VideoTexture> videoTexture;

	class Scene* parent;

	bool hidden;//default visibility
private:
	Model();//private ctr
	ci::gl::Texture gl_texture;
};
}
#endif //MODEL_H