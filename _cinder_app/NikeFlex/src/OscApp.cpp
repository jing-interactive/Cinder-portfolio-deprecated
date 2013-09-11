#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct OscApp : public AppBasic 
{
    static const int kPort = 3000;
    
    void prepareSettings(Settings *settings)
    {
        settings->setTitle("flex");
        settings->setWindowPos(0, 0);
        settings->setWindowSize(640, 480);
    }

    void setup()
    {
        mListener.setup(kPort);
    //  sender.setup();
    }

    void update()
    {
        while (mListener.hasWaitingMessages())
        {
            osc::Message msg;
            mListener.getNextMessage(&msg);
            const string& addr = msg.getAddress();

            if (addr == "/pull")
            {
                int file_idx = msg.getArgAsInt32(0);
            }
        }
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
