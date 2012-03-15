#include <cinder/Utilities.h>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Spark.h"
#include "Config.h"

void StateSparkInteractive::enter()
{
	n_countdown = SPARKINT_COUNTDOWN;
	printf("%d %s\n", _dev_id, "SparkInteractive");
	resetTimer();
	items = new Spark[SPARKINT_N_ITEMS];
}

void StateSparkInteractive::update()
{
	vector<Vec3i> centers;
	bool updated = _app.getNewCenter(centers, _dev_id);

	for (int i=0;i<SPARKINT_N_ITEMS;i++)
	{
		if (updated)
		{ 
			items[i].setCenter(centers[0]);
		}
		items[i].update(_dev_id);
		items[i].life = 1;//hack..
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
