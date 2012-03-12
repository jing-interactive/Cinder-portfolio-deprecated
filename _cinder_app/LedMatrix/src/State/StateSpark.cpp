#include <cinder/Utilities.h>
#include <vector>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Spark.h"

using namespace std;

namespace
{
	const int n_items = 100;//duplicates might exist
}

void StateSpark::enter()
{	
	n_countdown = 60;
	printf("%d %s\n", _dev_id, "[idle]Spark");
	resetTimer();
	items = new Spark[n_items];
}

void StateSpark::update()
{
	for (int i=0;i<n_items;i++)
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
	LedManager::get(_dev_id).fadeOut(2);
	delete[] items;
}
