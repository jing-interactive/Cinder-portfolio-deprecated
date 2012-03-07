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
	typedef Map::value_type  Pair;
	typedef Map::iterator Iter;
	typedef Map::const_iterator ConstIter;

	Map _contents;
public:
	std::shared_ptr<class Content> getContentByName(const std::string& name) const;
	bool load(const std::string& plist);
};
}
#endif //CONTENT_MANAGER_H