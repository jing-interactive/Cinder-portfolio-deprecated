#include "Function.h"
#include <boost/algorithm/string.hpp>
#include <vector>
#include "Functions.h"

using namespace std;

namespace ARContent{

	//gotolink::::http://www.youtube.com/watch?v=TWSDaF73sOc
	//gotoscene::::default
Function* Function::create( const std::string& on_click )
{
	Function* fun = NULL;
	vector<string> words;
	boost::algorithm::split(words, on_click, boost::is_any_of("#"),	boost::algorithm::token_compress_on );
	//create different Function according to words
	if (words[0] == "gotolink")
	{
		fun = new FunctionGotoLink();
		fun->args[0] = words[2];
	}
	else if (words[1] == "gotoscene")
	{
		fun = new FunctionGotoScene();
		fun->args[0] = words[1];
	}
	return fun;
}

Function* Function::create( const std::string& function, const std::string& arg1, const std::string& arg2/*=""*/, const std::string& arg3/*=""*/ )
{
	return NULL;
}

}