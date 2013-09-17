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

osc::Listener   mListener;

struct Led
{
    Led(const Vec2f& pos) : mPos(pos)
    {
        mValue = mTargetValue = 0.0f;
        mIsDirty = false;
    }

    void setValue(float aValue)
    {
        mTargetValue = aValue;
        mIsDirty = true;
    }

    float getValue()
    {
        if (mIsDirty)
        {
            mValue = constrain(lerp(mValue, mTargetValue, LED_LERP_FACTOR), 0.0f, 1.0f);
        }
        return mValue;
    }

    Vec2f   mPos;

private:
    float   mValue;
    float   mTargetValue;
    bool    mIsDirty;
};
vector<Led>     mLeds;

bool            mOscDirty;
vector<Vec2f>   mVisitors;
vector<Vec2f>   mVisitorsTemp;

#define GET_INSTANCE_IMPL(classname) \
    static Ref getInstance()\
    {\
        static Ref sInstance = Ref(new classname);\
        return sInstance;\
    }

struct FirstState : public State<OscApp>
{
    GET_INSTANCE_IMPL(FirstState);

    void enter(OscApp* app);
};

struct OscApp : public AppBasic, StateMachine<OscApp>
{
    OscApp() : StateMachine<OscApp>(this) 
    {

    }

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
            Vec2f center(
                cx * getWindowWidth(), cy * getWindowHeight());

            mLeds.push_back(Led(center));
        }

        mListener.registerMessageReceived(this, &OscApp::onOscMessage);

        changeToState(FirstState::getInstance());
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
            mOscDirty = true;
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
        if (mOscDirty)
        {
            mVisitors = mVisitorsTemp;
            mOscDirty = false;
        }

        updateIt();
    }

    void draw()
    {
        gl::clear(ColorA::black());

        gl::color(Color::white());
        for (size_t k=0; k<mVisitors.size(); k++)
        {
            gl::drawStrokedEllipse(mVisitors[k], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }

        for (size_t i=0; i<mLeds.size(); i++)
        {
            gl::color(Color(0, 0, mLeds[i].getValue()));
            gl::drawStrokedEllipse(mLeds[i].mPos, VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }

        drawIt();
    }

private:

};

struct InteractiveState : public State<OscApp>
{
    GET_INSTANCE_IMPL(InteractiveState);

    void update(OscApp* app)
    {
        for (size_t i=0; i<mLeds.size(); i++)
        {
            float sum = 0;
            for (size_t k=0; k<mVisitors.size(); k++)
            {
                float dist = EFFECTIVE_RADIUS / mLeds[i].mPos.distance(mVisitors[k]);
                sum += dist * dist;
            }
            mLeds[i].setValue(sum);
        }

        if (false)
        {
            app->changeToState(FirstState::getInstance());
        }        
    }
};

struct InteractiveMaxState : public State<OscApp>
{
    GET_INSTANCE_IMPL(InteractiveMaxState);

    void enter(OscApp* app)
    {

    }

    void update(OscApp* app)
    {

    }
};

struct IdleBulletState : public State<OscApp>
{
    GET_INSTANCE_IMPL(IdleBulletState);

    void update(OscApp* app)
    {

    }
};

struct IdleSparkState : public State<OscApp>
{
    GET_INSTANCE_IMPL(IdleSparkState);

    void update(OscApp* app)
    {

    }
};

struct IdleWTFState : public State<OscApp>
{
    GET_INSTANCE_IMPL(IdleWTFState);

    void update(OscApp* app)
    {

    }
};

void FirstState::enter(OscApp* app)
{
    app->changeToState(InteractiveMaxState::getInstance());
}

CINDER_APP_BASIC(OscApp, RendererGl)
