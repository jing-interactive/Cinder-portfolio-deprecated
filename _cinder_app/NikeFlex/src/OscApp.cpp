#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"
#include "../../../_common/MiniConfig.h"
#include "../../../_common/StateMachine.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct OscApp;

// global variables
osc::Listener   mListener;
osc::Sender     mSender;
vector<Vec2f>   mScenePositions;
vector<float>   mScenePowers;

bool            mDirty;
vector<Vec2f>   mVisitors;
vector<Vec2f>   mVisitorsTemp;

struct InteractiveState : public State<OscApp>
{
    InteractiveState(OscApp& app): State<OscApp>(app){}

    void update();

    void draw()
    {
        gl::color(Color::white());
        for (size_t k=0; k<mVisitors.size(); k++)
        {
            gl::drawStrokedEllipse(mVisitors[k], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }

        for (size_t i=0; i<mScenePositions.size(); i++)
        {
            mScenePowers[i] = 0;
            for (size_t k=0; k<mVisitors.size(); k++)
            {
                float dist = EFFECTIVE_RADIUS / mScenePositions[i].distance(mVisitors[k]);
                mScenePowers[i] += dist * dist;
            }

            gl::color(Color(0, 0, constrain(mScenePowers[i], 0.0f, 1.0f)));
            gl::drawStrokedEllipse(mScenePositions[i], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }
    }
};

struct InteractiveMaxState : public State<OscApp>
{
    InteractiveMaxState(OscApp& app): State<OscApp>(app){}

    void enter()
    {

    }

    void draw()
    {

    }
};

struct AnimationState : public State<OscApp>
{
    AnimationState(OscApp& app): State<OscApp>(app){}
    void draw()
    {

    }
};

struct OscApp : public AppBasic, StateMachine<OscApp>
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
            mScenePositions.push_back(center);
        }
        mScenePowers.resize(mScenePositions.size());

        mListener.registerMessageReceived(this, &OscApp::onOscMessage);

        mStateInteractive = StateRef(new InteractiveState(*this));
        mStateAnimation = StateRef(new AnimationState(*this));

        changeToState(mStateInteractive);
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

        updateIt();
    }

    void draw()
    {
        gl::clear(ColorA::black());

        drawIt();
    }

public:
    StateRef        mStateInteractive;
    StateRef        mStateInteractiveMax;
    StateRef        mStateAnimation;
};

void InteractiveState::update()
{
    if (false)
    {
        mObj.changeToState(mObj.mStateInteractiveMax);
    }
}

CINDER_APP_BASIC(OscApp, RendererGl)
