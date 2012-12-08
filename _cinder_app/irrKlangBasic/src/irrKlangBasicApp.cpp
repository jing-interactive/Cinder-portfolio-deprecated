#include "cinder/app/AppBasic.h"
#include "cinder/audio/Output.h"
#include "cinder/audio/Io.h"
#include "cinder/audio/SourceFileWindowsMedia.h"
#include "cinder/Timeline.h"

#include "Resources.h"

using namespace ci;
using namespace ci::app;

#define CATCH_EXCEPTION( ExceptionType ) \
	catch ( ExceptionType& e ){\
	console() << #ExceptionType << e.what() << std::endl;}


class irrKlangBasicApp : public AppBasic {
public:
	void setup()
	{
		//loadAudio( "getout.ogg" );
		loadAudio( "ophelia.mp3" );
		mAudioTracks.push_back( audio::Output::addTrack( mAudioSources.front() ) );
		mVolume = 0.0f;
		timeline().apply( &mVolume, 1.0f, 4.0f, easeInOutQuad ).pingPong().loop();
	}

	void update()
	{
		console() << mVolume << std::endl;
		mAudioTracks.front()->setVolume( mVolume );
	}

	void draw()
	{
		gl::clear();
		gl::drawSolidCircle( Vec2f( getWindowWidth() / 2, mVolume * getWindowHeight() ), 10 );
	}

private:
	void loadAudio(const std::string& filename)
	{
		try
		{
			audio::SourceRef src = audio::load( loadAsset( filename ) );
			if ( src )
				mAudioSources.push_back( src );
		}
		CATCH_EXCEPTION( AssetLoadExc )
		CATCH_EXCEPTION( audio::IoException )
	}
	std::vector<audio::SourceRef> mAudioSources;
	std::vector<audio::TrackRef> mAudioTracks;
	Anim<float> mVolume;
};

CINDER_APP_BASIC( irrKlangBasicApp, RendererGl )
