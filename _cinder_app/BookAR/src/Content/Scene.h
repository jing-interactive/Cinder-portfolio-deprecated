#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include <cinder/Cinder.h>

namespace ARContent{
class Scene
{
public:
	std::string name;
	
	//TODO: 
	//http://blippar.com/service/ar/tas/metroie/video/playSound2.php?sound=trex.wav::::Loading...
	std::string initialCommands;

	std::vector<std::shared_ptr<class Model>> models;
};
}
#endif //SCENE_H