#include "Content.h"
#include <cinder/Xml.h>

using namespace std;

namespace ARContent{

	Content* Content::create( const cinder::XmlTree& xmltree )
	{
		string key = xmltree.getValue<string>();
		int a = 0;

		return NULL;
	}
}