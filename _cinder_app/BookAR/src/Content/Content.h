#ifndef CONTENT_H
#define CONTENT_H

#include <vector>
#include <string>
#include <cinder/Cinder.h>

namespace ARContent{

class Content
{
public:
	enum Type
	{
		T_OVERLAY,
		T_3D,
		T_LINK,//TODO: link is pointer to other Content
	};
	//it's not the KEY(mdl file name), it's the Content name
	//KEY is not saved in Content, cause multiple mdl files can point to one Content
	//
	//<key>name</key>
	//<string>Jurassic Park</string>
	std::string name;

	bool sync;//yes/no

	Type type;

	std::vector<std::shared_ptr<class Scene>> _scenes;
};
}
#endif //CONTENT_H