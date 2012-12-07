#include "cinder/app/AppBasic.h"
#include "cinder/audio/Output.h"
#include "cinder/audio/Io.h"
#include "cinder/audio/SourceFileWindowsMedia.h"

#include "Resources.h"

using namespace ci;
using namespace ci::app;

#define CATCH_EXCEPTION( ExceptionType ) \
	catch ( ExceptionType& e ){\
	console() << #ExceptionType << e.what() << std::endl;}


class irrKlangBasicApp : public AppBasic {
public:
	void setup();
	void mouseDown( MouseEvent );
	void draw();

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
};

void irrKlangBasicApp::setup()
{
	//loadAudio( "getout.ogg" );
	loadAudio( "ophelia.mp3" );
}

void irrKlangBasicApp::mouseDown( MouseEvent event )
{
	audio::Output::play( mAudioSources.front() );
}

void irrKlangBasicApp::draw()
{
	glClearColor( 0.1f, 0.1f, 0.1f, 1.0f );
	glClear( GL_COLOR_BUFFER_BIT );
}

CINDER_APP_BASIC( irrKlangBasicApp, RendererGl )
