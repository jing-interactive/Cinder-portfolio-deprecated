#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"
#include "Spark.h"
#include <vector>

using namespace std;

namespace
{
	const int n_countdown = 60;
	const int n_sparks = 100;//duplicates might exist
}

void StateSpark::enter()
{
	resetTimer();
	sparks = new Spark[n_sparks];
}

void StateSpark::update()
{
	for (int i=0;i<n_sparks;i++)
	{
		sparks[i].update(_dev_id);
	}
	if (getElapsedSeconds() > n_countdown)
		_app.changeToRandomIdleState(_dev_id);
}

void StateSpark::draw()
{

}

void StateSpark::exit()
{
	delete[] sparks;
}
