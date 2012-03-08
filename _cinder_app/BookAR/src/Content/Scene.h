#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include <cinder/Cinder.h>
#include <cinder/Matrix.h>

namespace cinder{
	class XmlTree;
}

namespace ARContent{
class Scene
{
public:
	static Scene* create(const cinder::XmlTree& xmltree);
	std::string name;

	void setMatrix(const ci::Matrix44d& modelview,	const ci::Matrix44d& proj, const ci::Vec2f corners[4]);

	void draw();

	class Content* parent;

	//rendering part
	ci::Matrix44d modelview;
	ci::Matrix44d proj;
	ci::Vec2f	 corners[4];

	//TODO: 
	//http://blippar.com/service/ar/tas/metroie/video/playSound2.php?sound=trex.wav::::Loading...
	std::string initialCommands;

	std::vector<std::shared_ptr<class Model>> models;
private:
	Scene();
};

}
#endif //SCENE_H