#include "LedMatrixApp.h"
#include <cinder/MayaCamUI.h>
#include "LedState.h"

void LedMatrixApp::mouseDown( MouseEvent event )
{
	maya_cam->mouseDown( event.getPos() );
}

void LedMatrixApp::mouseDrag( MouseEvent event )
{
	maya_cam->mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void LedMatrixApp::keyUp( KeyEvent event )
{
	static StateType idle_states[3] = {T_SPARK,T_BREATHE,T_LOTS};
	static StateType interative_states[3] = {T_FOLLOWING,T_SPARK_INT,T_ANIMAL};
	int key_id = -1;
	
	switch (event.getCode())
	{
	case KeyEvent::KEY_SPACE:
		{
			show_3d = !show_3d;
		}break;
	case KeyEvent::KEY_1:
		{
			key_id = 0;
		}break;
	case KeyEvent::KEY_2:
		{
			key_id = 1;
		}break;
	case KeyEvent::KEY_3:
		{
			key_id = 2;
		}break;
	default:
		break;
	}

	if (key_id >= 0 && key_id <= 2)
	{
		bool idle = LedState::isIdleState(current_states[0]->_type);
		if (idle)
			changeToState(LedState::create(*this, 0, idle_states[key_id]));
		else
			changeToState(LedState::create(*this, 0, interative_states[key_id]));
	}
}
