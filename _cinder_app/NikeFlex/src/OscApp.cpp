// fix Spark µƒŒ Ã‚

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

bool            gOscDirty = false;
vector<Vec2f>   gVisitors;
vector<Vec2f>   gVisitorsTemp;

osc::Listener   gListener;
bool            gIsInteractiveState = false;
float           gInteractionEndingTime = 0;

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
            if (gIsInteractiveState)
            {
                mValue = constrain(lerp(mValue, mTargetValue, INTERACTIVE_LERP_FACTOR), 0.0f, 1.0f);
            }
            else
            {
                mValue = constrain(lerp(mValue, mTargetValue, LED_LERP_FACTOR), 0.0f, 1.0f);
            }
        }
        return mValue;
    }

    float getDistanceToScene()
    {
        float sum = 0;
        for (size_t k=0; k<gVisitors.size(); k++)
        {
            float dist = mPos.distance(gVisitors[k]);
            float val = 0;
            if (dist > MAX_EFFECTIVE_RADIUS)
            {
                val = 0;
            }
            else if (dist < MIN_EFFECTIVE_RADIUS)
            {
                val = 1.0f;
            }
            else
            {
                val = 1.0f - dist / MAX_EFFECTIVE_RADIUS;
            }
            sum += val;
        }
        return sum;
    }

    Vec2f   mPos;
    float   mBirthTime;
    float   mTargetValue;

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
        settings->setWindowSize(640, 640);

        readConfig();
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
            gl::drawStrokedEllipse(gVisitors[k], VIRTUAL_CIRCLE_RADIUS, VIRTUAL_CIRCLE_RADIUS);
        }

        for (size_t i=0; i<gLeds.size(); i++)
        {
            float value = gLeds[i].getValue();
            //value = constrain(value * COLOR_MULTIPLIER, 0.0f, 1.0f);
            gl::color(Color(value, value, value));
            gl::drawSolidEllipse(gLeds[i].mPos, VIRTUAL_CIRCLE_RADIUS, VIRTUAL_CIRCLE_RADIUS);
            gl::drawPoint(Vec2f(i, 0));
        }

        drawIt();

        params::InterfaceGl::draw();
    }
};

struct MyState : public State<OscApp>
{
    void enterShared(OscApp* app)
    {
        mBirthSeconds = getElapsedSeconds();
        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].setValue(0.0f);
        }
    }

    static Ref getRandomIdleState();

    void tryChangeIdleState(OscApp* app);

    float mBirthSeconds;
};

struct StateMaxInteractive : public MyState
{
    GET_SINGLETON_IMPL(StateMaxInteractive);

    void enter(OscApp* app)
    {
        console() << "Enter StateMaxInteractive" << endl;
        gIsInteractiveState = true;
        enterShared(app);
        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].mBirthTime = getElapsedSeconds();
            gLeds[i].setValue(0.0f);
        }
    }

    void update(OscApp* host);
};

struct StateInteractive : public MyState
{
    GET_SINGLETON_IMPL(StateInteractive);

    void enter(OscApp* app)
    {
        console() << "Enter StateInteractive" << endl;
        gIsInteractiveState = true;
        enterShared(app);
    }

    void update(OscApp* app)
    {
        float totalSum = 0;
        float maxSum = 0;
        for (size_t i=0; i<gLeds.size(); i++)
        {
            Led& led = gLeds[i];
            float sum = led.getDistanceToScene();
            maxSum = max(maxSum, sum);
            if (sum <= IDLE_TO_INTERACTIVE_THRESH)
            {
                led.mBirthTime = getElapsedSeconds();
            }
            led.setValue(sum);
            totalSum += led.getValue();
        }
#ifdef _DEBUG
        console() << "maxSum: " << maxSum << endl;
#endif
        if (totalSum > MAX_POWER_PERCENT * gLeds.size())
        {
            app->changeToState(StateMaxInteractive::getSingleton());
        }
    }
};

void StateMaxInteractive::update( OscApp* host )
{
    for (size_t i=0; i<gLeds.size(); i++)
    {
        float value = MAX_AMPLIFIER * abs(sin((getElapsedSeconds() - gLeds[i].mBirthTime) * MAX_SIN_FACTOR));
        gLeds[i].mValue = gLeds[i].mTargetValue = value + 0.1f;
    }
    //console() << gLeds[0].mValue << endl;
}

struct StateTransmit : public MyState
{
    GET_SINGLETON_IMPL(StateTransmit);

    MyState::Ref mNextState;
    void setRealState(MyState::Ref nextState)
    {
        mNextState = nextState;
    }

    void enter(OscApp* app)
    {
        console() << "Enter StateTransmit" << endl;
        gIsInteractiveState = false;
        enterShared(app);
    }

    void update(OscApp* app)
    {
        bool niceToChange = true;
        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].setValue(gLeds[i].mValue - 0.1f);
            if (gLeds[i].mValue > 0.0f)
            {
                niceToChange = false;
            }
        }
        if (niceToChange)
        {
            app->changeToState(mNextState);
        }
    }
};

struct StateIdleBullet : public MyState
{
    GET_SINGLETON_IMPL(StateIdleBullet);

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleBullet" << endl;
        gIsInteractiveState = false;
        enterShared(app);
    }

    void update(OscApp* app)
    {
        float elpased = getElapsedSeconds() - mBirthSeconds;

        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].setValue(gLeds[i].mTargetValue - elpased * BULLET_FADEOUT_SPEED);
        }

        size_t currentIndex = static_cast<size_t>(BULLET_SPEED * elpased) % gLeds.size();
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
        enterShared(app);
        mLastSpark = getElapsedSeconds();
        mSpeeds.resize(gLeds.size());
    }

    void update(OscApp* app)
    {
        float elpased = getElapsedSeconds() - mBirthSeconds;

        for (int i = 0; i < gLeds.size(); i++)
        {
            gLeds[i].setValue(gLeds[i].mTargetValue + (getElapsedSeconds() - gLeds[i].mBirthTime) * mSpeeds[i]);

            if (mSpeeds[i] > FLT_EPSILON)
            {
                if (gLeds[i].getValue() > 0.95f)
                {
                    gLeds[i].mBirthTime = getElapsedSeconds();
                    mSpeeds[i] = -SPARK_FADEOUT_SPEED;
                }
            }
            else if (mSpeeds[i] < -FLT_EPSILON)
            {
                // fade out
                //if (gLeds[i].getValue() < 0.05f)
                //{
                //    gLeds[i].mBirthTime = getElapsedSeconds();
                //    mSpeeds[i] = -SPARK_FADEOUT_SPEED;
                //}
            }
        }

        if (getElapsedSeconds() - mLastSpark > SPARK_INTERVAL)
        {
            mLastSpark = getElapsedSeconds();

            size_t idx = rand() % gLeds.size();

            size_t minId = max<int>(0,               idx);
            size_t maxId = min<int>(gLeds.size(),    idx + SPARK_COUNT);
            for (size_t i=minId; i<maxId; i++)
            {
                if (gLeds[i].getValue() < FLT_EPSILON)
                {
                    gLeds[i].setValue(0.0f);
                    gLeds[i].mBirthTime = getElapsedSeconds();
                    mSpeeds[i] = SPARK_FADEIN_SPEED;
                }
                //gLeds[i].setValue(1.0f);
            }
        }

        tryChangeIdleState(app);
    }

    vector<float> mSpeeds;
    float mLastSpark;
};

struct StateIdleDragon : public MyState
{
    GET_SINGLETON_IMPL(StateIdleDragon);

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleDragon" << endl;
        gIsInteractiveState = false;
        enterShared(app);
        mLastDragon = 0;
        mCurrentGroup = 0;
        mCount = 0;
        mStartId = 0;
        mEndId = 0;
    }

    void update(OscApp* app)
    {
        float elpased = getElapsedSeconds() - mBirthSeconds;

        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].setValue(gLeds[i].mTargetValue * (1.0f - DRAGON_FADEOUT_SPEED) );
        }

        if (getElapsedSeconds() - mLastDragon > DRAGON_INTERVAL)
        {
            mLastDragon = getElapsedSeconds();
            if (mCount == 0 || mCount == 2)
            {
                mCount = 1;
                if (mCount == 2)
                    mCount = 0;
                mStartId = rand() % gLeds.size();
                mEndId = rand() % gLeds.size();

                if (mStartId > mEndId)
                {
                    swap(mStartId, mEndId);
                }

                if (mEndId - mStartId < 4)
                {
                    mEndId = min<int>(mStartId + randInt(4, 8), gLeds.size() - 1);
                }
            }
            else
            {
                mCount = 2;
                swap(mStartId, mEndId);
            }
        }

        int idx = lerp(mStartId, mEndId, (getElapsedSeconds() - mLastDragon) / DRAGON_INTERVAL);
        gLeds[idx].setValue(1.0f);

        tryChangeIdleState(app);
    }

    float mLastDragon; 
    int mCurrentGroup;
    int mStartId;
    int mEndId;
    int mCurrentId;
    float mTimeSlice;
    int mCount; // 0/1/2
};

struct StateIdleBlocky : public MyState
{
    GET_SINGLETON_IMPL(StateIdleBlocky);

    vector<float> mSpeed;

    void enter(OscApp* app)
    {
        console() << "Enter StateIdleBlocky" << endl;
        gIsInteractiveState = false;
        enterShared(app);
        mSpeed.resize(gLeds.size());

        count = 0;
    }

    float time;

    void update(OscApp* app)
    {
        float elpased = getElapsedSeconds() - time;

        if (count == 0)
        {
            time = getElapsedSeconds();

            count = 1;
            for (size_t i=0; i<gLeds.size(); i++)
            {
                gLeds[i].mValue = gLeds[i].mTargetValue = 0.0f;
                mSpeed[i] = randFloat(BLOCKY_FADEOUT_MINSPEED, BLOCKY_FADEOUT_MAXSPEED) * 0.3f;
            }
        }

        for (size_t i=0; i<gLeds.size(); i++)
        {
            gLeds[i].setValue(gLeds[i].mTargetValue + elpased * mSpeed[i]);
        }

        tryChangeIdleState(app);

        float sum = 0;
        for (size_t i=0; i<gLeds.size(); i++)
        {
            sum += gLeds[i].getValue();
        }
        if (count == 2 && sum < 0.5f)
        {
            count = 0;
        }
        else if (count == 1 && sum > gLeds.size() * 0.8f)
        {
            count = 2;
            time = getElapsedSeconds();

            for (size_t i=0; i<gLeds.size(); i++)
            {
                //gLeds[i].mValue = gLeds[i].mTargetValue = 1.0f;
                mSpeed[i] = -randFloat(BLOCKY_FADEOUT_MINSPEED, BLOCKY_FADEOUT_MAXSPEED) * 0.3f;
            }
        }
    }

    int count;
};

MyState::Ref MyState::getRandomIdleState()
{
    switch (rand() % 4)
    {
        case 0: return StateIdleBullet::getSingleton();
        case 1: return StateIdleDragon::getSingleton();
        case 2: return StateIdleBlocky::getSingleton();
        case 3: return StateIdleSpark::getSingleton();
        default: throw Exception();
    }
}

void MyState::tryChangeIdleState( OscApp* app )
{
    if (!gIsInteractiveState)
    {
        if (getElapsedSeconds() - mBirthSeconds > IDLE_SWITCH_PERIOD)
        {
            ((StateTransmit*)(StateTransmit::getSingleton().get()))->setRealState(getRandomIdleState());
            app->changeToState(StateTransmit::getSingleton());
        }
    }
}


void OscApp::setup()
{
    gParams = params::InterfaceGl("param", Vec2i(300, 450));
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
    float sum = 0;
    for (size_t i=0; i<gLeds.size(); i++)
    {
        Led& led = gLeds[i];
        sum += led.getDistanceToScene();
    }
    if (sum > IDLE_TO_INTERACTIVE_THRESH)
    {
        gIsInteractiveState = true;
        gInteractionEndingTime = getElapsedSeconds();
    }
    else
    {
        if (getElapsedSeconds() - gInteractionEndingTime > IDLE_TO_INTERACTIVE_SECONDS)
        {
            gIsInteractiveState = false;
        }
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
            changeToState(StateIdleDragon::getSingleton());
        }break;
    case KeyEvent::KEY_6:
        {
            changeToState(StateIdleBlocky::getSingleton());
        }break;
    default: break;
    }
}

CINDER_APP_BASIC(OscApp, RendererGl)
