#include "PuzzleApp.h"
#include "OscListener.h"
#include "cinder/ImageIo.h"
#include "cinder/Rand.h"
#include "Hand.h"

void PuzzleApp::prepareSettings(Settings *settings){
	settings->setFrameRate(60.0f);
	settings->setWindowSize(848, 564);
	settings->setResizable(false);
}

void PuzzleApp::setup()
{
	_next_z = 0;
	_rotate = 0;

	Color8u hand_clrs[N_HANDS]={Color8u(255,122,122),Color8u(100,0,255)};
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
