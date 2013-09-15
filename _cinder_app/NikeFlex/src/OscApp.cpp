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
    void prepareSettings(Settings *settings)
    {
        settings->setTitle("flex");
        settings->setWindowPos(0, 0);
        settings->setWindowSize(640, 480);

        readConfig();
    }

    void keyUp(KeyEvent event)
    {
        if (event.getCode() == KeyEvent::KEY_SPACE)
        {
            readConfig();
            return;
        }

        if (event.getCode() == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void setup()
    {
        mListener.setup(OSC_PORT);
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
        mSceneLights.resize(mScenePlots.size());

        mListener.registerMessageReceived(this, &OscApp::onOscMessage);
    }

    void onOscMessage(const osc::Message* msg)
    {
        const string& addr = msg->getAddress();

        if (addr == "/start")
        {
            mVisitorsTemp.clear();
            return;
        }

        if (addr == "/end")
        {
            mDirty = true;
            return;
        }

        if (addr == "/contour")
        {
            float cx = msg->getArgAsFloat(2) * getWindowWidth();
            float cy = msg->getArgAsFloat(3) * getWindowHeight();
            mVisitorsTemp.push_back(Vec2f(cx, cy));

            return;
        }
    }

    void update()
    {
        if (mDirty)
        {
            mVisitors = mVisitorsTemp;
            mDirty = false;
        }
    }

    void draw()
    {
        gl::clear(ColorA::black());

        gl::color(Color::white());
        for (size_t k=0; k<mVisitors.size(); k++)
        {
            gl::drawStrokedEllipse(mVisitors[k], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }

        for (size_t i=0; i<mScenePlots.size(); i++)
        {
            mSceneLights[i] = 0;
            for (size_t k=0; k<mVisitors.size(); k++)
            {
                float dist = EFFECTIVE_RADIUS / mScenePlots[i].distance(mVisitors[k]);
                mSceneLights[i] += dist * dist;
            }

            gl::color(Color(0, 0, constrain(mSceneLights[i], 0.0f, 1.0f)));
            gl::drawStrokedEllipse(mScenePlots[i], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }
    }

private:
    osc::Listener   mListener;
    osc::Sender     mSender;
    vector<Vec2f>   mScenePlots;
    vector<float>   mSceneLights;

    bool            mDirty;
    vector<Vec2f>   mVisitors;
    vector<Vec2f>   mVisitorsTemp;
};

CINDER_APP_BASIC(OscApp, RendererGl)
