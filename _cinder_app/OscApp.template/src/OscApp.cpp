#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"
#include "../../../_common/MiniConfig.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#pragma warning(disable: 4244)

struct OscApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        settings->setWindowPos(0, 0);
        settings->setWindowSize(640, 480);

        readConfig();
    }

    void setup()
    {
        mListener.setup(OSC_PORT);
    //  sender.setup();
        mListener.registerMessageReceived(this, &OscApp::onOscMessage);
    }

    void keyUp(KeyEvent event)
    {
        if (event.getCode() == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void onOscMessage(const osc::Message* msg)
    {
        const string& addr = msg->getAddress();

        if (addr == "/pull")
        {
            int file_idx = msg->getArgAsInt32(0);
        }
    }

    void update()
    {
    
    }

    void draw()
    {
        gl::clear(ColorA::black());
    }

private:
    osc::Listener   mListener;
    osc::Sender     mSender;
};

CINDER_APP_BASIC(OscApp, RendererGl)
