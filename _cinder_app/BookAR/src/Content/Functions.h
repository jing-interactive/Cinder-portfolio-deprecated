#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "Function.h"

namespace ARContent{

class FunctionShow : public Function
{
public:
	void execute();
};

// <key>function</key>
// <string>hide</string>
// <key>arg1</key>
// <string>heinz_button_sorry</string>
class FunctionHide : public Function
{
public:
	void execute();
};

class FunctionGotoScene : public Function
{
public:
	void execute();
};

class FunctionGotoLink : public Function
{
public:
	void execute();
};

class FunctionPlayInAppVideo : public Function
{
public:
	void execute();
};

class FunctionBindAnimation : public Function
{
public:
	void execute();
};

class FunctionPlaySound : public Function
{
public:
	void execute();
};

class FunctionSetVar : public Function
{
public:
	void execute();
};

class FunctionShowOverlay : public Function
{
public:
	void execute();
};

//two types of calling

//loadcommands::::http://blippar.com/service/ar/tas/heinz/checkWinner.php::::Loading...

// <key>function</key>
// <string>loadCommands</string>
// <key>arg1</key>
// <string>http://blippar.com/service/ar/tas/kitkat/submit.php?id={{intVar1}}</string>
// <key>arg2</key>
// <string>Voting, please wait...</string>
class FunctionLoadCommands : public Function
{
public:
	void execute();
};

// <key>function</key>
// <string>lockLandscape</string>
// <key>arg1</key>
// <string>1000</string>
class FunctionLockLandscape : public Function
{
public:
	void execute();
};

}
#endif //FUNCTIONS_H