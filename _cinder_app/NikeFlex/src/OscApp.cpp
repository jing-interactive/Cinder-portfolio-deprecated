#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"
#include "cinder/Rand.h"
#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"
#include "../../../_common/StateMachine.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct OscApp;

namespace cinder { namespace gl {
    void drawPoint( const Vec2f &pt )
    {
        drawSolidRect( Rectf( pt.x, pt.y, pt.x+1, pt.y+1 ) );
    }
} }

struct Led
{
    Led(const Vec2f& pos) : mPos(pos)
    {
        mValue = mTargetValue = 0.0f;
        mIsDirty = false;
        mBirthTime = getElapsedSeconds();
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
    float   mBirthTime;
    float   mTargetValue;

private:
    float   mValue;
    bool    mIsDirty;
};
vector<Led>     mLeds;

bool            mOscDirty = false;
vector<Vec2f>   mVisitors;
vector<Vec2f>   mVisitorsTemp;

osc::Listener   mListener;
bool            mIsInteractiveStates = false;

params::InterfaceGl mParams;

struct OscApp : public AppBasic, StateMachine<OscApp>
{
    OscApp() : StateMachine<OscApp>(this) 
    {

    }

    void prepareSettings(Settings *settings)
    {
        settings->setTitle("flex");
        settings->setBorderless();
        settings->setWindowPos(0, 0);
        settings->setWindowSize(640, 480);

        readConfig();
    }

    void keyDown(KeyEvent event)
    {
        if (event.getCode() == KeyEvent::KEY_d)
        {
            return;
        }
    }

    void keyUp(KeyEvent event);

    void setup();

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

        if (IS_DEBUG_MODE)
        {
            mVisitors.clear();
            mVisitors.push_back(getMousePos());
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
            float wave = mLeds[i].getValue();
            if (mIsInteractiveStates)
            {
                wave *= abs(sin(getElapsedSeconds() - mLeds[i].mBirthTime) * LED_SIN_FACTOR);
            }
            gl::color(Color(0, 0, wave));
            gl::drawSolidEllipse(mLeds[i].mPos, VIRTUAL_RADIUS, VIRTUAL_RADIUS);
            gl::drawPoint(Vec2f(i, 0));
        }

        drawIt();

        params::InterfaceGl::draw();
    }
};

#define GET_INSTANCE_IMPL(classname) \
    static Ref getInstance()\
{\
    static Ref sInstance = Ref(new classname);\
    return sInstance;\
}

struct StateInteractive : public State<OscApp>
{
    GET_INSTANCE_IMPL(StateInteractive);

    void enter(OscApp* app)
    {
        console() << "Enter StateInteractive" << endl;
        mIsInteractiveStates = true;
    }

    void update(OscApp* app)
    {
        float totalSum = 0;
        for (size_t i=0; i<mLeds.size(); i++)
        {
            Led& led = mLeds[i];
            float sum = 0;
            for (size_t k=0; k<mVisitors.size(); k++)
            {
                float dist = EFFECTIVE_RADIUS / led.mPos.distance(mVisitors[k]);
                sum += dist * dist;
            }
            const float kSumThreshold = 0.1f;
            if (sum <= kSumThreshold)
            {
                led.mBirthTime = getElapsedSeconds();
            }
            led.setValue(sum);
            totalSum += sum;
        }

#ifdef MAX_STATE_ENABLED
        if (totalSum > MAX_POWER_RATIO * mLeds.size())
        {
            app->changeToState(StateMaxInteractive::getInstance());
        }
#endif // MAX_STATE_ENABLED
    }
};

struct StateMaxInteractive : public State<OscApp>
{
    GET_INSTANCE_IMPL(StateMaxInteractive);

    void enter(OscApp* app)
    {
        console() << "Enter StateMaxInteractive" << endl;
        mIsInteractiveStates = true;
    }
};

struct StateIdleBullet : public State<OscApp>
{
    GET_INSTANCE_IMPL(StateIdleBullet);

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleBullet" << endl;
        mIsInteractiveStates = false;
        mBirthTime = getElapsedSeconds();
    }

    void update(OscApp* app)
    {
        float elpased = getElapsedSeconds() - mBirthTime;

        for (size_t i=0; i<mLeds.size(); i++)
        {
            mLeds[i].setValue(mLeds[i].mTargetValue - elpased * IDLE_BULLET_FADEOUT_SPEED);
        }

        size_t currentIndex = static_cast<size_t>(IDLE_BULLET_SPEED * elpased) % mLeds.size();
        mLeds[currentIndex].setValue(1.0f);
    }

    float   mBirthTime;
};

struct StateIdleSpark : public State<OscApp>
{
    GET_INSTANCE_IMPL(StateIdleSpark);

    void setup(OscApp* app)
    {
        console() << "Enter StateIdleSpark" << endl;
        mIsInteractiveStates = false;
        mLastGenerateTime = getElapsedSeconds();
    }

    void update(OscApp* app)
    {
        if (getElapsedSeconds() - mLastGenerateTime > IDLE_SPARK_INTERVAL)
        {
            for (int i = 0; i < IDLE_SPARK_COUNT; i++)
            {
                mLeds[rand() % mLeds.size()].setValue(randFloat());
            }
            mLastGenerateTime = getElapsedSeconds();
        }
    }

    float   mLastGenerateTime;
};

struct StateIdleWTF : public State<OscApp>
{
    GET_INSTANCE_IMPL(StateIdleWTF);

    void setup(OscApp* app)
    {
        console() << "Enter StateIdleWTF" << endl;
        mIsInteractiveStates = false;
    }

    void update(OscApp* app)
    {

    }
};

#undef GET_INSTANCE_IMPL

void OscApp::setup()
{
    mParams = params::InterfaceGl("param", Vec2i(300, 300));
    setupConfigUI(&mParams);

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
        mLeds.push_back(Led(center));
    }

    mListener.registerMessageReceived(this, &OscApp::onOscMessage);

    changeToState(StateIdleBullet::getInstance());
}

void OscApp::keyUp(KeyEvent event)
{
    switch (event.getCode())
    {
    case KeyEvent::KEY_SPACE:
        {
            readConfig();
        }break;
    case KeyEvent::KEY_ESCAPE:
        {
            quit();
        }break;
    case KeyEvent::KEY_1:
        {
            changeToState(StateInteractive::getInstance());
        }break;
    case KeyEvent::KEY_2:
        {
            changeToState(StateMaxInteractive::getInstance());
        }break;
    case KeyEvent::KEY_3:
        {
            changeToState(StateIdleBullet::getInstance());
        }break;
    case KeyEvent::KEY_4:
        {
            changeToState(StateIdleSpark::getInstance());
        }break;
    case KeyEvent::KEY_5:
        {
            changeToState(StateIdleWTF::getInstance());
        }break;
    default: break;
    }
}

CINDER_APP_BASIC(OscApp, RendererGl)
