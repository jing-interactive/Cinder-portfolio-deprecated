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

class TuioGateway : public AppBasic {
public:
//     void prepareSettings( Settings *settings )
//     {
//         settings->enableMultiTouch();
//     }

    void mouseDown( MouseEvent event )
    {
        mouseDrag(event);
    }

    void mouseUp( MouseEvent event )
    {
        mCursorPressed = false;
    }

    void mouseDrag( MouseEvent event )
    {
        mCursorPressed = true;
        mCursorPos = event.getPos();
    }

    enum {USAGE_CLIENT, USAGE_SERVER, USAGE_ROUTER, USAGE_COUNT};

    void onConnect()
    {
        APP_USAGE = math<int>::clamp(APP_USAGE, USAGE_CLIENT, USAGE_ROUTER);

        mCursorPressed = false;

        mTuioClient.disconnect();
        mTuioServer = osc::Sender();
        mOscServer = osc::Sender();

        HWND hWnd = getRenderer()->getHwnd();
        const char* kUsageDescs[] = 
        {
            "TuioGateway - Client Mode",
            "TuioGateway - Server Mode",
            "TuioGateway - Router Mode",
        };
        ::SetWindowTextA( hWnd,  kUsageDescs[APP_USAGE]);

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

    void setup()
    {
        console() << "TuioGateway built on " << __DATE__ << endl;

        readConfig();
        mStatus = "idle..press CONNECT button";
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
  
        //mTuioClient.registerTouches( this );
        //console() << "MT: " << System::hasMultiTouch() << " Max points: " << System::getMaxMultiTouchPoints() << std::endl;

        mFont = Font("YouYuan", 22);
    }

    void update()
    {
        N_DISPLAYS = math<int>::clamp(N_DISPLAYS, 1, 8);
        REMOTE_DISPLAY_ID = math<int>::clamp(REMOTE_DISPLAY_ID, 1, N_DISPLAYS);

        mActiveTouches = mTuioClient.getCursors();

        if (mCursorPressed)
        {
            mActiveTouches.push_back(tuio::Cursor(
                "", -1, Vec2f(mCursorPos.x/getWindowWidth(), mCursorPos.y/getWindowHeight())
                ));
        }

        if (mCurrentAppUsage == USAGE_SERVER || mCurrentAppUsage == USAGE_ROUTER)
        {
            sendTuioMessage(mTuioServer, mActiveTouches);
            sendOscMessages(mOscServer, mActiveTouches);
        }
    }

    void draw()
    {
        gl::enableAlphaBlending();
        gl::clear( Color( 0.1f, 0.1f, 0.1f ) );
        gl::setMatricesWindow( getWindowWidth(), getWindowHeight() );

        if (VISUAL_EFFECT)
        {
            // draw yellow circles at the active touch points
            gl::color( Color( 1, 1, 0 ) );

            for( vector<tuio::Cursor>::const_iterator touchIt = mActiveTouches.begin(); touchIt != mActiveTouches.end(); ++touchIt )
                gl::drawStrokedCircle( Vec2f(touchIt->getPos().x * getWindowWidth(), touchIt->getPos().y * getWindowHeight()),20.0f );

            gl::drawString(mStatus, Vec2f(10, getWindowHeight() - 100), ColorA::white(), mFont);
        }

        mParams.draw();
    }

    void keyDown( KeyEvent event ) {
        switch (event.getCode())
        {
        case KeyEvent::KEY_ESCAPE:
            {
                quit();
            }break;
        case KeyEvent::KEY_h:
            {
                if (mParams.isVisible())
                {
                    mParams.show( false );
                    VISUAL_EFFECT = false;
                }
                else
                {
                    mParams.show( true );
                    VISUAL_EFFECT = true;
                }
            }
        default:
            break;
        }
    }

    void sendTuioMessage( osc::Sender& sender, const vector<tuio::Cursor>& cursors ) 
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

        if (!cursors.empty())
        {
            float cellSize = 1.0f/N_DISPLAYS;
            float x0 = cellSize * (REMOTE_DISPLAY_ID - 1);
            float x1 = cellSize * (REMOTE_DISPLAY_ID);

            for (vector<tuio::Cursor>::const_iterator it = cursors.begin(); it != cursors.end(); ++it)
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

    void sendOscMessages( osc::Sender& sender, const vector<tuio::Cursor>& cursors ) 
    {
        if (!cursors.empty())
        {
            float cellSize = 1.0f/N_DISPLAYS;
            float x0 = cellSize * (REMOTE_DISPLAY_ID - 1);
            float x1 = cellSize * (REMOTE_DISPLAY_ID);

            int cursorIdx = 0;
            for (vector<tuio::Cursor>::const_iterator it = cursors.begin(); it != cursors.end(); ++it, cursorIdx++)
            {
                if (cursorIdx == MAX_CURSOR_COUNT)
                    break;

                float x = it->getPos().x;
                float y = it->getPos().y;

                x = x0 + x * cellSize;
                //y = y0 + y * cellSize;

                if (x <= x0 || x >= x1 || y <= 0 || y >= 1.0f)
                    continue;

                std::string addr = "/cursor/" + toString(cursorIdx);
                osc::Bundle bundle;
                {
                    osc::Message m;
                    m.setAddress(addr + "/x");
                    m.addFloatArg(x);
                    bundle.addMessage(m);
                }
                {
                    osc::Message m;
                    m.setAddress(addr + "/y");
                    m.addFloatArg(y);
                    bundle.addMessage(m);
                }
                sender.sendBundle(bundle);
            }
        }
    }

private:
    params::InterfaceGl         mParams;

    tuio::Client				mTuioClient;
    osc::Sender                 mTuioServer;
    osc::Sender                 mOscServer;

    vector<string>              mEnumTypes;
    string                      mStatus;
    Font                        mFont;

    mutex                       mMutex;
    bool                        mCursorPressed;
    Vec2f                       mCursorPos;

    int                         mCurrentAppUsage;
    vector<tuio::Cursor>        mActiveTouches;
};

CINDER_APP_BASIC( TuioGateway, RendererGl )

