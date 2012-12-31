#include "cinder/Cinder.h"
#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/System.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"
#include "cinder/tuio/TuioClient.h"
#include "cinder/osc/OscSender.h"
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

        if (mLine.size() > 0)
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

    enum {USAGE_CLIENT, USAGE_SERVER, USAGE_GATEWAY, USAGE_COUNT};

    void onConnect()
    {
        APP_USAGE = math<int>::clamp(APP_USAGE, USAGE_CLIENT, USAGE_GATEWAY);

        mClearJobScheduled = true;
        mCursorPressed = false;

        mTuioClient.disconnect();
        mTuioServer = osc::Sender();
        mOscServer = osc::Sender();

        char buffer[MAX_PATH] = {0};

        strcpy(buffer, mEnumTypes[APP_USAGE].c_str());

#define CATCH_ERROR  catch (std::exception& e)   {\
            sprintf_s(buffer, MAX_PATH, "%s | [FAIL] %s!!!", buffer, e.what());\
            console() << __FILE__ << " (" << __LINE__ << ") " << e.what() << endl;\
            mCurrentAppUsage = USAGE_COUNT;\
            break;}

        do 
        {
            if (APP_USAGE != USAGE_SERVER)
            {
                try
                {
                    mTuioClient.connect(LOCAL_TUIO_PORT);
                    sprintf_s(buffer, MAX_PATH, "%s | listen at #%d", 
                        buffer, LOCAL_TUIO_PORT);
                    mCurrentAppUsage = APP_USAGE;
                }
                CATCH_ERROR
            }

            if (APP_USAGE != USAGE_CLIENT)
            {
                try
                {
                    mTuioServer.setup(REMOTE_IP, REMOTE_TUIO_PORT);
                    mOscServer.setup(REMOTE_IP, REMOTE_OSC_PORT);
                    sprintf_s(buffer, MAX_PATH , "%s | sending to %s: #%d | #%d", 
                        buffer, REMOTE_IP.c_str(), REMOTE_TUIO_PORT, REMOTE_OSC_PORT);
                    mCurrentAppUsage = APP_USAGE;
                }
                CATCH_ERROR
            }
        } while (0);

#undef CATCH_ERROR

        mStatus = buffer;
    }

    void	setup()
    {
        console() << "TuioGateway built on " << __DATE__ << endl;

        readConfig();
        mStatus = "idle..press CONNECT button";
        mClearJobScheduled = true;
        mCurrentAppUsage = USAGE_COUNT;

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

        mFont = Font("YouYuan", 22);
    }

    void update()
    {
        N_DISPLAYS = math<int>::clamp(N_DISPLAYS, 1, 8);
        REMOTE_DISPLAY_ID = math<int>::clamp(REMOTE_DISPLAY_ID, 1, N_DISPLAYS);

        if (mClearJobScheduled)
        {
            mClearJobScheduled = false;
            mActivePoints.clear();
            mDyingPoints.clear();
        }
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
        vector<tuio::Cursor> activeTouches( mTuioClient.getCursors() );

        if (mCursorPressed)
        {
            activeTouches.push_back(tuio::Cursor(
                "", -1, Vec2f(mCursorPos.x/getWindowWidth(), mCursorPos.y/getWindowHeight())
                ));
            mPrevCursorPos = mCursorPos;
        }

        if (mCurrentAppUsage == USAGE_SERVER || mCurrentAppUsage == USAGE_GATEWAY)
            sendTuioMessages(mTuioServer, activeTouches);

        for( vector<tuio::Cursor>::const_iterator touchIt = activeTouches.begin(); touchIt != activeTouches.end(); ++touchIt )
            gl::drawStrokedCircle( Vec2f(touchIt->getPos().x * getWindowWidth(), touchIt->getPos().y * getWindowHeight()),20.0f );

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

    void sendTuioMessages( osc::Sender& sender, const vector<tuio::Cursor>& activeTouches ) 
    {
        osc::Bundle b;
        osc::Message alive;
        // Sends alive message - saying 'Hey, there's no alive blobs'
        alive.setAddress("/tuio/2Dcur");
        alive.addStringArg("alive");

        // Send fseq message
        osc::Message fseq;
        fseq.setAddress( "/tuio/2Dcur" );
        fseq.addStringArg( "fseq" );
        fseq.addIntArg(getElapsedFrames());

        float cellSize = 1.0f/N_DISPLAYS;
        float x0 = cellSize * (REMOTE_DISPLAY_ID - 1);
        float x1 = cellSize * (REMOTE_DISPLAY_ID);

        if (!activeTouches.empty())
        {
            for (vector<tuio::Cursor>::const_iterator it = activeTouches.begin(); it != activeTouches.end(); ++it)
            {
                float x = it->getPos().x;
                float y = it->getPos().y;

                x = x0 + x * cellSize;
                //y = y0 + y * cellSize;

                if (x <= x0 || x >= x1 || y <= 0 || y >= 1.0f)
                    continue;

                osc::Message set;
                set.setAddress( "/tuio/2Dcur" );
                set.addStringArg("set");
                set.addIntArg(it->getSessionId());				// id
                set.addFloatArg(x);	// x
                set.addFloatArg(y);	// y
                set.addFloatArg(it->getSpeed().x);			// dX
                set.addFloatArg(it->getSpeed().y);			// dY
                set.addFloatArg(it->getMotionAccel());		// m

                b.addMessage( set );							// add message to bundle
                alive.addIntArg(it->getSessionId());				// add blob to list of ALL active IDs
            }
        }
        b.addMessage( alive );		// add message to bundle
        b.addMessage( fseq );		// add message to bundle

        try
        {
            sender.sendBundle( b ); // send bundle
        }
        catch (std::exception& e)
        {
            console() << "sendBundle : " << e.what() << endl;
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
    bool                        mClearJobScheduled;
    int                         mCurrentAppUsage;
};

CINDER_APP_BASIC( TuioGateway, RendererGl )

