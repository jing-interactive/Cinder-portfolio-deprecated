#include "BookAR.h"
#include <cinder/params/Params.h>

void BookAR::draw()
{
	StateMachine<BookAR>::draw();

	params::InterfaceGl::draw();
}
