#include "cinder/Cinder.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/System.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"
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

class TuioGateway : public AppBasic {
public:
    void	prepareSettings( Settings *settings )
    {
        settings->enableMultiTouch();
    }

    void    mouseDown( MouseEvent event )
    {
        mouseDrag(event);
        mPrevCursorPos = mCursorPos;
    }

    void    mouseUp( MouseEvent event )
    {
        mCursorPressed = false;
    }

    void    mouseDrag( MouseEvent event )
    {
        mCursorPressed = true;
        mCursorPos = event.getPos();
    }


    void	touchesBegan( TouchEvent event )
    {
        //console() << "Began: " << event << std::endl;

        for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt ) {
            Color newColor( CM_HSV, Rand::randFloat(), 1, 1 );
            mActivePoints.insert( make_pair( touchIt->getId(), TouchPoint( touchIt->getPos(), newColor ) ) );
        }
    }

    void	touchesMoved( TouchEvent event )
    {
        //console() << "Moved: " << event << std::endl;

        for( vector<TouchEvent::Touch>::const_iterator touchIt = event.getTouches().begin(); touchIt != event.getTouches().end(); ++touchIt )
            mActivePoints[touchIt->getId()].addPoint( touchIt->getPos() );
    }

    void	touchesEnded( TouchEvent event )
    {
        //console() << "Ended: " << event << std::endl;

        lock_guard<mutex> locker(mMutex);
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

        mActivePoints.clear();
        mCursorPressed = false;

        mTuioClient.disconnect();
        mTuioServer = osc::Sender();
        mOscServer = osc::Sender();

        char buffer[MAX_PATH] = {0};

        switch (APP_USAGE)
        {
        case USAGE_CLIENT:
            {
                mTuioClient.connect(LOCAL_TUIO_PORT);
                sprintf_s(buffer, MAX_PATH, "%s : listening at #%d", 
                    mEnumTypes[APP_USAGE].c_str(), LOCAL_TUIO_PORT);
            }break;
        case USAGE_SERVER:
            {
                mTuioServer.setup(REMOTE_IP, REMOTE_TUIO_PORT);
                mTuioServer.setup(REMOTE_IP, REMOTE_OSC_PORT);
                sprintf_s(buffer, MAX_PATH , "%s : sending to %s: #%d | #%d", 
                    mEnumTypes[APP_USAGE].c_str(), REMOTE_IP.c_str(), REMOTE_TUIO_PORT, REMOTE_OSC_PORT);
                mStatus = buffer;
            }break;
        case USAGE_GATEWAY:
            {
                mTuioClient.connect(LOCAL_TUIO_PORT);
                mTuioServer.setup(REMOTE_IP, REMOTE_TUIO_PORT);
                mOscServer.setup(REMOTE_IP, REMOTE_OSC_PORT);
                sprintf_s(buffer, MAX_PATH, "%s : listening at %d, sending to %s: #%d | #%d", 
                    mEnumTypes[APP_USAGE].c_str(), LOCAL_TUIO_PORT, REMOTE_IP.c_str(), REMOTE_TUIO_PORT, REMOTE_OSC_PORT);
            }break;
        default:
            break;
        }
        mStatus = buffer;
    }

    void	setup()
    {
        readConfig();
        mStatus = "idle..press CONNECT button";

        mParams = params::InterfaceGl("param", Vec2i(270, 240));
        {
            mEnumTypes.clear();
            mEnumTypes.push_back("Client");
            mEnumTypes.push_back("Server");
            mEnumTypes.push_back("Router");

            // MAGIC!
            int lines = 0;
#define ITEM_DEF(type, var, default) mParams.addParam(#var, &var);
#include "item.def"
#undef ITEM_DEF

            // HACK!
            mParams.removeParam("APP_USAGE");
            mParams.addSeparator();
            mParams.addParam("APP_USAGE", mEnumTypes, &APP_USAGE);
            mParams.addSeparator();
            mParams.addButton("CONNECT", std::bind(&TuioGateway::onConnect, this));
            mParams.addButton("SAVE", writeConfig);

            onConnect();
        }
  
        mTuioClient.registerTouches( this );

        console() << "MT: " << System::hasMultiTouch() << " Max points: " << System::getMaxMultiTouchPoints() << std::endl;

        mFont = Font("YouYuan", 24);
    }

    void	draw()
    {
        gl::enableAlphaBlending();
        gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
        gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );

        {
            lock_guard<mutex> locker(mMutex);
            for( map<uint32_t,TouchPoint>::const_iterator activeIt = mActivePoints.begin(); activeIt != mActivePoints.end(); ++activeIt ) {
                activeIt->second.draw();
            }
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

        if (mCursorPressed)
        {
            activeTouches.push_back(TouchEvent::Touch(mCursorPos, mPrevCursorPos, -1, getElapsedSeconds(), NULL));
            mPrevCursorPos = mCursorPos;
        }

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

    mutex                       mMutex;
    bool                        mCursorPressed;
    Vec2f                       mCursorPos, mPrevCursorPos;
};

CINDER_APP_BASIC( TuioGateway, RendererGl )

