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
	printf("%d %s\n", _dev_id, "SparkInteractive");
	resetTimer();
	sparks = new Spark[n_sparks];
}

void StateSparkInteractive::update()
{
	Vec3i center;
	bool updated = _app.getNewCenter(center, _dev_id);
	for (int i=0;i<n_sparks;i++)
	{
		if (updated)
			sparks[i].setCenter(center);
		sparks[i].update(_dev_id);
	}
}

void StateSparkInteractive::draw()
{

}

void StateSparkInteractive::exit()
{
	delete[] sparks;
}
