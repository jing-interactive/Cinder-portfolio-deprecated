#ifndef FUNCTION_H
#define FUNCTION_H

#include <string>

namespace ARContent{

class Function
{
public:
	enum{
		N_ARGS = 3,
	};
	static Function* create(const std::string& on_click);
	static Function* create(const std::string& function, const std::string& arg1, const std::string& arg2="", const std::string& arg3="");

	virtual void execute() = 0;

	std::string args[N_ARGS];
};
}
#endif //FUNCTION_H