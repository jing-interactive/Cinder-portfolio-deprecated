#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"
#include "../../../_common/MiniConfig.h"

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
        ifstream ifs(getAssetPath("scene.plots").string().c_str());
        if (!ifs)
        {
            console() << "Failed to find scene.plots!" << endl;
            quit();
        }

        float cx, cy;
        while (ifs >> cx >> cy)
        {
            Vec2f center(cx * getWindowWidth(), cy * getWindowHeight());
            mScenePlots.push_back(center);
        }

        mListener.registerMessageReceived(this, &OscApp::onOscMessage);
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
        for (int i=0; i<mScenePlots.size(); i++)
        {
            gl::drawStrokedEllipse(mScenePlots[i], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }
    }

private:
    osc::Listener   mListener;
    osc::Sender     mSender;
    vector<Vec2f>   mScenePlots;
};

CINDER_APP_BASIC(OscApp, RendererGl)
