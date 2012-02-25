#include "PuzzleApp.h"
#include "OscMessage.h"

void PuzzleApp::onOscMessage( const osc::Message* msg )
{
	if (msg->getNumArgs() != 3)
		return;
	string addr = msg->getAddress();
	string action = msg->getArgAsString(0);
	console()<<addr<<action<<endl;
	int x = getWindowWidth()*msg->getArgAsFloat(1);
	int y = getWindowHeight()*msg->getArgAsFloat(2);

	int id = (addr == "/left") ? LEFT : RIGHT;
	_hands[id].pos.set(x,y);
	_hands[id].push = (action == "push");
}

void PuzzleApp::update()
{ 
	updateStates();
}