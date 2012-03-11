#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"
#include "Spark.h"

namespace
{
	const int n_countdown = 2;
	const int n_sparks = 20;
}

void StateSparkInteractive::enter()
{
	resetTimer();
	sparks = new Spark[n_sparks];
}

void StateSparkInteractive::update()
{
	Vec3i center;//be integer
	bool centers_updated = !_app.centers[_dev_id].empty();
	if (centers_updated)
		center = _app.centers[_dev_id].front();
	for (int i=0;i<n_sparks;i++)
	{
		sparks[i].update(_dev_id);
		if (centers_updated)
			sparks[i].setCenter(center);
	}
}

void StateSparkInteractive::draw()
{
//	gl::color(1,1,1);
	// gl::draw(_app._tex_selected);
	// gl::drawStringCentered(welcome, pos, clr, font);
}

void StateSparkInteractive::exit()
{
	delete[] sparks;
}
