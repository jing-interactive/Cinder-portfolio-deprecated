#include "cinder/Cinder.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/System.h"
#include "cinder/Rand.h"
#include "TuioClient.h"
#include "OscSender.h"
#include "cinder/params/Params.h"
#include "MiniConfig.h"

using namespace ci;
using namespace ci::app;

#include <vector>
#include <map>
#include <list>
using namespace std;

struct TouchPoint {
    TouchPoint() {}
    TouchPoint( const Vec2f &initialPt, const Color &color ) : mColor( color ), mTimeOfDeath( -1.0 ) 
    {
        mLine.push_back( initialPt ); 
    }

    void addPoint( const Vec2f &pt ) { mLine.push_back( pt ); }

    void draw() const
    {
        if( mTimeOfDeath > 0 ) // are we dying? then fade out
            gl::color( ColorA( mColor, ( mTimeOfDeath - getElapsedSeconds() ) / 2.0f ) );
        else
            gl::color( mColor );

        gl::draw( mLine );
    }

    void startDying() { mTimeOfDeath = getElapsedSeconds() + 2.0f; } // two seconds till dead

    bool isDead() const { return getElapsedSeconds() > mTimeOfDeath; }

    PolyLine<Vec2f>	mLine;
    Color			mColor;
    float			mTimeOfDeath;
};

// We'll create a new Cinder Application by deriving from the BasicApp class
class MultiTouchApp : public AppBasic {
public:
    void	prepareSettings( Settings *settings )
    {
        settings->enableMultiTouch();
    }

    void	touchesBegan( TouchEvent event )
    {
        console() << "Began: " << event << std::endl;
        for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt ) {
            Color newColor( CM_HSV, Rand::randFloat(), 1, 1 );
            mActivePoints.insert( make_pair( touchIt->getId(), TouchPoint( touchIt->getPos(), newColor ) ) );
        }
    }

    void	touchesMoved( TouchEvent event )
    {
        console() << "Moved: " << event << std::endl;
        for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt )
            mActivePoints[touchIt->getId()].addPoint( touchIt->getPos() );
    }

    void	touchesEnded( TouchEvent event )
    {
        console() << "Ended: " << event << std::endl;
        for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt ) {
            mActivePoints[touchIt->getId()].startDying();
            mDyingPoints.push_back( mActivePoints[touchIt->getId()] );
            mActivePoints.erase( touchIt->getId() );
        }
    }

    enum {USAGE_CLIENT, USAGE_SERVER, USAGE_GATEWAY};

    void onConnect()
    {
        math<int>::clamp(APP_USAGE, USAGE_CLIENT, USAGE_GATEWAY);

        if (APP_USAGE != USAGE_SERVER)
        {
            mTuioClient.disconnect();
            mTuioClient.connect(LOCAL_TUIO_PORT);
        }
        if (APP_USAGE != USAGE_CLIENT)
        {
            mTuioServer.setup(REMOTE_IP, REMOTE_TUIO_PORT);
            mOscServer.setup(REMOTE_IP, REMOTE_OSC_PORT);
        }
        mStatus = mEnumTypes[APP_USAGE] +" Mode is running.";
    }

    void	setup()
    {
        readConfig();
        mStatus = "idle..press CONNECT button";

        mParams = params::InterfaceGl("param", Vec2i(250, 190));
        {
            mEnumTypes.clear();
            mEnumTypes.push_back("Client");
            mEnumTypes.push_back("Server");
            mEnumTypes.push_back("Router");

            // MAGIC!
#define ITEM_DEF(type, var, default) mParams.addParam(#var, &var);
#include "item.def"
#undef ITEM_DEF

            // HACK!
            mParams.removeParam("APP_USAGE");
            mParams.addSeparator();
            mParams.addParam("APP_USAGE", mEnumTypes, &APP_USAGE);
            mParams.addSeparator();
            mParams.addButton("CONNECT", std::bind(&MultiTouchApp::onConnect, this));
            mParams.addButton("SAVE", writeConfig);

            onConnect();
        }
  
        mTuioClient.registerTouches( this );

        console() << "MT: " << System::hasMultiTouch() << " Max points: " << System::getMaxMultiTouchPoints() << std::endl;

        mFont = Font("YouYuan", 32);
    }

    void	draw()
    {
        gl::enableAlphaBlending();
        gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
        gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );

        for( map<uint32_t,TouchPoint>::const_iterator activeIt = mActivePoints.begin(); activeIt != mActivePoints.end(); ++activeIt ) {
            activeIt->second.draw();
        }

        for( list<TouchPoint>::iterator dyingIt = mDyingPoints.begin(); dyingIt != mDyingPoints.end(); ) {
            dyingIt->draw();
            if( dyingIt->isDead() )
                dyingIt = mDyingPoints.erase( dyingIt );
            else
                ++dyingIt;
        }

        // draw yellow circles at the active touch points
        gl::color( Color( 1, 1, 0 ) );
        vector<TouchEvent::Touch> activeTouches( mTuioClient.getActiveTouches() );
        for( vector<TouchEvent::Touch>::const_iterator touchIt = activeTouches.begin(); touchIt != activeTouches.end(); ++touchIt )
            gl::drawStrokedCircle( touchIt->getPos(), 20.0f );

        gl::drawString(mStatus, Vec2f(10, getWindowHeight() - 100), ColorA::white(), mFont);
        mParams.draw();
    }

    void	keyDown( KeyEvent event ) {
        switch (event.getCode())
        {
        case KeyEvent::KEY_SPACE:
            {
                setFullScreen( ! isFullScreen() ); 
            }break;
        case KeyEvent::KEY_ESCAPE:
            {
                quit();
            }break;
        default:
            break;
        }
    }

private:
    map<uint32_t,TouchPoint>	mActivePoints;
    list<TouchPoint>			mDyingPoints;
 
    params::InterfaceGl         mParams;

    tuio::Client				mTuioClient;
    osc::Sender                 mTuioServer;
    osc::Sender                 mOscServer;

    vector<string>              mEnumTypes;
    string                      mStatus;
    Font                        mFont;
};

CINDER_APP_BASIC( MultiTouchApp, RendererGl )

