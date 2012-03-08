#include "Function.h"
#include <vector>
#include "Functions.h"
#include <cinder/Utilities.h>

using namespace std;

namespace ARContent{

	//gotolink::::http://www.youtube.com/watch?v=TWSDaF73sOc
	//gotoscene::::default
Function* Function::create( const std::string& on_click )
{
	Function* fun = NULL;
	vector<string> words = ci::split(on_click, ':');
	if (words.size() > 2)
	{
		//create different Function according to words
		if (words[0] == "gotolink")
		{
			fun = new FunctionGotoLink();
			fun->args[0] = words[1]+":"+words[2];
		}
		else if (words[1] == "gotoscene")
		{
			fun = new FunctionGotoScene();
			fun->args[0] = words[1];
		}
	}

	return fun;
}

Function* Function::create( const std::string& function, const std::string& arg1, const std::string& arg2/*=""*/, const std::string& arg3/*=""*/ )
{
	return NULL;
}

Function::Function()
{
	parent = NULL;
}

}