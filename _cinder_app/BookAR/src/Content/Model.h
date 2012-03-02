#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <string>
#include <cinder/Cinder.h>

namespace cinder{
	class XmlTree;
}

namespace ARContent{

class Model
{
public:
	//OPTIONAL
	std::vector<std::shared_ptr<class Animation>> animations;

	std::string name;
	std::string name_type;//always "MD2"

	//gotolink::::http://www.youtube.com/watch?v=TWSDaF73sOc
	//TODO: on_click and click_actions won't exist together, they might be combined
	std::shared_ptr<class Function> on_click;
	std::vector<std::shared_ptr<class Function>> click_actions;

	float rotation_x,rotation_y,rotation_z;
	float scale;

	std::string texture;
	std::string texture_type;

	std::shared_ptr<class VideoTexture> videoTexture;

	bool hidden;//default visibility
};
}
#endif //MODEL_H