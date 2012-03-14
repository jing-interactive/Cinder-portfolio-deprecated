#include <cinder/Utilities.h>
#include <vector>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Spark.h"
#include "Config.h"

using namespace std;

void StateSpark::enter()
{	
	n_countdown = SPARK_COUNTDOWN;
	printf("%d %s\n", _dev_id, "[idle]Spark");
	resetTimer();
	items = new Spark[SPARK_N_ITEMS];
}

void StateSpark::update()
{
	for (int i=0;i<SPARK_N_ITEMS;i++)
	{
		items[i].update(_dev_id);
	}
	LedState::update();
}

void StateSpark::draw()
{

}

void StateSpark::exit()
{
	delete[] items;
}
