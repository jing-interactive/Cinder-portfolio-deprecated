#ifndef CONTENT_MANAGER_H
#define CONTENT_MANAGER_H

#include <map>
#include <string>
#include <cinder/Cinder.h>

namespace ARContent{

class ContentManager
{
public:
	typedef std::map<std::string, std::shared_ptr<class Content>> Map;
	typedef std::pair<std::string, std::shared_ptr<class Content>> Pair;
	typedef Map::iterator Iter;

	Map _contents;
public:
	bool load(const std::string& plist);
};
}
#endif //CONTENT_MANAGER_H