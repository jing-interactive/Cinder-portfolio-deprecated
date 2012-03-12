#include <cinder/Utilities.h>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Spark.h"

namespace
{
	const int n_sparks = 20;
}

void StateSparkInteractive::enter()
{
	n_countdown = 2;
	printf("%d %s\n", _dev_id, "SparkInteractive");
	resetTimer();
	items = new Spark[n_sparks];
}

void StateSparkInteractive::update()
{
	Vec3i center;
	bool updated = _app.getNewCenter(center, _dev_id);
	for (int i=0;i<n_sparks;i++)
	{
		if (updated)
			items[i].setCenter(center);
		items[i].update(_dev_id);
	}

	LedState::update();
}

void StateSparkInteractive::draw()
{

}

void StateSparkInteractive::exit()
{
	delete[] items;
}
