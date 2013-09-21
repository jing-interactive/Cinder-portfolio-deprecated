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

const float kSumThreshold = 0.1f;

bool            gOscDirty = false;
vector<Vec2f>   gVisitors;
vector<Vec2f>   gVisitorsTemp;

osc::Listener   gListener;
bool            gIsInteractiveState = false;

params::InterfaceGl gParams;

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

    float getDistanceToScene()
    {
        float sum = 0;
        for (size_t k=0; k<gVisitors.size(); k++)
        {
            float dist = EFFECTIVE_RADIUS / mPos.distance(gVisitors[k]);
            sum += dist * dist;
        }
        return sum;
    }

    Vec2f   mPos;
    float   mBirthTime;
    float   mTargetValue;

private:
    float   mValue;
    bool    mIsDirty;
};
vector<Led>     gLeds;

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
            gVisitorsTemp.clear();
            return;
        }

        if (addr == "/end")
        {
            gOscDirty = true;
            return;
        }

        if (addr == "/contour")
        {
            float cx = msg->getArgAsFloat(2) * getWindowWidth();
            float cy = msg->getArgAsFloat(3) * getWindowHeight();
            gVisitorsTemp.push_back(Vec2f(cx, cy));

            return;
        }
    }

    void update();

    void draw()
    {
        gl::clear(ColorA::black());

        gl::color(Color::white());
        for (size_t k=0; k<gVisitors.size(); k++)
        {
            gl::drawStrokedEllipse(gVisitors[k], VIRTUAL_RADIUS, VIRTUAL_RADIUS);
        }

        for (size_t i=0; i<gLeds.size(); i++)
        {
            float value = gLeds[i].getValue();
            if (gIsInteractiveState)
            {
                value *= abs(sin(getElapsedSeconds() - gLeds[i].mBirthTime) * LED_SIN_FACTOR);
            }
            gl::color(Color(0, 0, value));
            gl::drawSolidEllipse(gLeds[i].mPos, VIRTUAL_RADIUS, VIRTUAL_RADIUS);
            gl::drawPoint(Vec2f(i, 0));
        }

        drawIt();

        params::InterfaceGl::draw();
    }
};

struct MyState : public State<OscApp>
{
    void updateLastSeconds(OscApp* app)
    {
        mLastSeconds = getElapsedSeconds();
    }

    static Ref getRandomIdleState();

    void tryChangeIdleState(OscApp* app)
    {
        if (!gIsInteractiveState)
        {
            if (getElapsedSeconds() - mLastSeconds > IDLE_SWITCH_PERIOD)
            {
                app->changeToState(getRandomIdleState());
            }
        }
    }

    float mLastSeconds;
};

struct StateInteractive : public MyState
{
    GET_SINGLETON_IMPL(StateInteractive);

    void enter(OscApp* app)
    {
        console() << "Enter StateInteractive" << endl;
        gIsInteractiveState = true;
        updateLastSeconds(app);
    }

    void update(OscApp* app)
    {
        float totalSum = 0;
        for (size_t i=0; i<gLeds.size(); i++)
        {
            Led& led = gLeds[i];
            float sum = led.getDistanceToScene();
            if (sum <= kSumThreshold)
            {
                led.mBirthTime = getElapsedSeconds();
            }
            led.setValue(sum);
            totalSum += sum;
        }

#ifdef MAX_STATE_ENABLED
        if (totalSum > MAX_POWER_RATIO * gLeds.size())
        {
            app->changeToState(StateMaxInteractive::getSingleton());
        }
#endif // MAX_STATE_ENABLED
    }
};

struct StateMaxInteractive : public MyState
{
    GET_SINGLETON_IMPL(StateMaxInteractive);

    void enter(OscApp* app)
    {
        console() << "Enter StateMaxInteractive" << endl;
        gIsInteractiveState = true;
        updateLastSeconds(app);
    }
};

struct StateIdleBullet : public MyState
{
    GET_SINGLETON_IMPL(StateIdleBullet);

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleBullet" << endl;
        gIsInteractiveState = false;
        updateLastSeconds(app);
    }

    void update(OscApp* app)
    {
        float elpased = getElapsedSeconds() - mLastSeconds;

        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].setValue(gLeds[i].mTargetValue - elpased * IDLE_BULLET_FADEOUT_SPEED);
        }

        size_t currentIndex = static_cast<size_t>(IDLE_BULLET_SPEED * elpased) % gLeds.size();
        gLeds[currentIndex].setValue(1.0f);

        tryChangeIdleState(app);
    }
};

struct StateIdleSpark : public MyState
{
    GET_SINGLETON_IMPL(StateIdleSpark);

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleSpark" << endl;
        gIsInteractiveState = false;
        updateLastSeconds(app);
    }

    void update(OscApp* app)
    {
        if (getElapsedSeconds() - mLastSeconds > IDLE_SPARK_INTERVAL)
        {
            for (int i = 0; i < IDLE_SPARK_COUNT; i++)
            {
                gLeds[rand() % gLeds.size()].setValue(randFloat());
            }
            updateLastSeconds(app);
        }

        tryChangeIdleState(app);
    }
};

struct StateIdleWTF : public MyState
{
    GET_SINGLETON_IMPL(StateIdleWTF);

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleWTF" << endl;
        gIsInteractiveState = false;
        updateLastSeconds(app);
    }

    void update(OscApp* app)
    {
        tryChangeIdleState(app);
    }
};

MyState::Ref MyState::getRandomIdleState()
{
    switch (rand() % 3)
    {
        case 0: return StateIdleBullet::getSingleton();
        case 1: return StateIdleSpark::getSingleton();
        case 2: return StateIdleWTF::getSingleton();
        default: throw Exception();
    }
}

void OscApp::setup()
{
    gParams = params::InterfaceGl("param", Vec2i(300, 300));
    setupConfigUI(&gParams);

    gListener.setup(OSC_PORT);
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
        gLeds.push_back(Led(center));
    }

    gListener.registerMessageReceived(this, &OscApp::onOscMessage);

    changeToState(StateIdleBullet::getSingleton());
}

void OscApp::update()
{
    if (gOscDirty)
    {
        gVisitors = gVisitorsTemp;
        gOscDirty = false;
    }

    if (IS_DEBUG_MODE)
    {
        gVisitors.clear();
        gVisitors.push_back(getMousePos());
    }

    bool isPrevInteractiveState = gIsInteractiveState;
    for (size_t i=0; i<gLeds.size(); i++)
    {
        Led& led = gLeds[i];
        float sum = led.getDistanceToScene();
        gIsInteractiveState = (sum > kSumThreshold);
    }

    if (!isPrevInteractiveState && gIsInteractiveState)
    {
        changeToState(StateInteractive::getSingleton());
    }
    else if (isPrevInteractiveState & !gIsInteractiveState)
    {
        changeToState(MyState::getRandomIdleState());
    }

    updateIt();
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
            changeToState(StateInteractive::getSingleton());
        }break;
    case KeyEvent::KEY_2:
        {
            changeToState(StateMaxInteractive::getSingleton());
        }break;
    case KeyEvent::KEY_3:
        {
            changeToState(StateIdleBullet::getSingleton());
        }break;
    case KeyEvent::KEY_4:
        {
            changeToState(StateIdleSpark::getSingleton());
        }break;
    case KeyEvent::KEY_5:
        {
            changeToState(StateIdleWTF::getSingleton());
        }break;
    default: break;
    }
}

CINDER_APP_BASIC(OscApp, RendererGl)
