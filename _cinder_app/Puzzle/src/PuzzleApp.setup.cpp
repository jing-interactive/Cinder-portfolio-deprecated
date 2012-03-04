#include "PuzzleApp.h"
#include "OscListener.h"
#include "cinder/ImageIo.h"
#include "cinder/Rand.h"
#include "Hand.h"
#include "cinder/Rand.h"

void PuzzleApp::prepareSettings(Settings *settings){
	settings->setFrameRate(60.0f);
	settings->setWindowSize(848, 564);
	settings->setResizable(false);
}

void PuzzleApp::setup()
{
	_next_z = 0;
	_rotate = 0;

	Rand::randomize();

	fnt_big = Font("STHupo", 64);
	fnt_small = Font("YouYuan", 32);

	ColorA hand_clrs[N_HANDS]={ColorA(1,0.5f,0.5f, 0.7f),ColorA(0.5f,0,1.0f,0.7f)};
	for (int i=0;i<N_HANDS;i++)
	{
		_hands[i] = shared_ptr<Hand>(new Hand(hand_clrs[i]));
		_hands[LEFT]->pos.set(ci::randInt(getWindowWidth()), ci::randInt(getWindowHeight()));
	}

	hideCursor();

	listener = shared_ptr<osc::Listener>(new osc::Listener());
	listener->setup(3333);
	listener->registerMessageReceived(this, &PuzzleApp::onOscMessage);

	fs::path mediaPath = getAppPath() / "media/";
	if ( fs::exists(mediaPath) && fs::is_directory(mediaPath))
	{
		fs::directory_iterator end;
		for( fs::directory_iterator iter(mediaPath) ; iter != end ; ++iter )
		{
			if (fs::is_regular_file(*iter))
			{
				_img_list.push_back(loadImage(*iter));
			}
		}
	}

	setupStates();
}
 
CINDER_APP_BASIC( PuzzleApp, RendererGl )
