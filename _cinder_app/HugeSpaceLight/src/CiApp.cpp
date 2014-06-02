#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/Xml.h"
#include "cinder/CinderMath.h"
#include "cinder/ip/Fill.h"
#include "cinder/Timeline.h"

#include "cinder/params/Params.h"

#include "cinder/Utilities.h"

#include "cinder/qtime/QuickTime.h"

#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"

#include "../../../_common/MiniConfig.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <list>

#include "../../../_common/AssetManager.h"
#include "../../../_common/StateMachine.h"

#include "GestureDetector.h"

#pragma warning(disable: 4244)

#pragma comment(lib, "CVClient.lib")
#pragma comment(lib, "QTMLClient.lib")

using namespace ci;
using namespace ci::app;
using namespace std;

const float kCamFov = 60.0f;
const int kOscPort = 4444;
const int kPadPort = 5555;
const int kKinectPort = 7001;
const float kLedOffset = 225.0f;
const Vec2i kGlobePhysicsSize(257, 18);
const Vec2i kWallPhysicsSize(21, 56);

// 
int             mElapsedLoopCount;
float           mRandomColorIndex;
int             mHour;
int             mCurrentHour;

Anim<float>     mGlobalAlpha = 1; // for fade-in / fade-out
float           mLastKinectMsgSeconds;

Channel         mIdleChannels[2];
float           mCurrentMovieTime;

int mPrevAnim = 0;

struct CiApp;

fs::directory_iterator kEndIt;

template <typename T>
static void updateTextureFrom(gl::Texture& tex, const T& surf)
{
    if (!surf) return;

    if (tex && surf.getSize() == tex.getSize())
        tex.update(surf);
    else
        tex = gl::Texture(surf);
}

static int getHour()
{
    SYSTEMTIME wtm;
    ::GetLocalTime(&wtm);
    return wtm.wHour;
}

// TODO: proto-buf?
struct AnimConfig
{
    static const int kCount = 11; // anim: [0, 9), kinect: 10
    static const int kKinect = kCount - 1;

    AnimConfig()
    {
        lightValue = 0.5f;
        lightValue2 = 0;
        loopCount = 1;
    }

    Color getColor() const
    {
        float value = 0;
        if (lightValue2 != 0)
        {
            float lv2 = lightValue2;
            float lv = lightValue;
            if (lv2 < lv)
            {
                std::swap<float>(lv, lv2);
            }
            const float range = math<float>::max(lv2 - lv, 0.001f);
            int k = mRandomColorIndex / range;
            value = (mRandomColorIndex - k * range) * range;
            if (k % 2 == 0)
            {
                value = lv + value;
            }
            else
            {
                value = lv2 - value;
            }
        }
        else
        {
            value = lightValue;
        }
        return Color(0.0f, value, 1.0f - value);
    }
    float lightValue;
    float lightValue2; // if non-zero, then random light value from (lightValue, lightValue2)
    // see getColor()
    int loopCount; // bigger than 1, or zero means don't play

    friend ostream& operator<<(ostream& lhs, const AnimConfig& rhs)
    {
        lhs << rhs.lightValue << " " << rhs.lightValue2 << " " << rhs.loopCount;
        return lhs;
    }

    friend istream& operator>>(istream& lhs, AnimConfig& rhs)
    {
        lhs >> rhs.lightValue >> std::ws >> rhs.lightValue2 >> std::ws >> rhs.loopCount;
        return lhs;
    }
};

struct Config
{
    static const int kCount = 6;

    AnimConfig animConfigs[AnimConfig::kCount];

    friend ostream& operator<<(ostream& lhs, const Config& rhs)
    {
        for (int i=0; i<AnimConfig::kCount; i++)
        {
            lhs << rhs.animConfigs[i] << " ";
        }
        return lhs;
    }

    friend istream& operator>>(istream& lhs, Config& rhs)
    {
        for (int i=0; i<AnimConfig::kCount; i++)
        {
            lhs >> rhs.animConfigs[i] >> std::ws;
        }
        return lhs;
    }
};

const string kProgSettingFileName = "ProgramSettings.xml";
const int kHourCount = 24; // valid hours for G9 are [10~23; 00; 01]

// to save key strokes

params::InterfaceGl mMainGUI;
params::InterfaceGl mProgramGUI;

osc::Listener   mPadListener;
osc::Listener   mKinectListener;
osc::Sender     mPadSender;

GestureDetector mGestures[24]; // TODO:

size_t gIdCount = 0;
struct Led
{
    Led(const Vec3f& aPos, float aValue = 1.0f): pos(aPos), value(aValue), id(gIdCount++){}
    Vec3f pos;
    Vec2i pos2d;
    float value;
    size_t id;
};

vector<qtime::MovieSurface>    mIdleAnims[2];

const size_t kKinectAnimTypeCount = 3;

struct KinectAnimSeq
{
    vector<Channel> seqs[2];
}mKinectAnimSequences[kKinectAnimTypeCount];

// TODO:
struct KinectAnim
{
    KinectAnim(float wavingSpeed)
    {
        mIsFinished = false;
        kinectSeq = &mKinectAnimSequences[rand() % kKinectAnimTypeCount];

        wavingSpeed = 24 * constrain(wavingSpeed * KINECT_MOVIE_SPEED, 1.0f, KINECT_MAX_SPEED);
        length = (float)kinectSeq->seqs[0].size() - 1;
        float duration = length / wavingSpeed;

        timeline().apply(&index, 0.0f, length, duration);
    }

    void get(Channel* globe, Channel* wall)
    {
        if (mIsFinished) return;

        *globe = kinectSeq->seqs[0][static_cast<int>(index)];
        *wall = kinectSeq->seqs[1][static_cast<int>(index)];
    }

    bool isFinished() const
    {
        return index.value() >= length;
    }

    float length;
    bool mIsFinished;
    KinectAnimSeq* kinectSeq;
    Anim<float> index;
};

list<KinectAnim> mKinectAnims;
Channel         mKinectSurfs[2]; // sum of mKinectAnims

int             mCurrentAnim;
int             ANIMATION;

int             mProbeConfig;
struct Config*  mCurrentConfig;

Color           mLedColor;

gl::Texture     mGlobeTexture, mWallTexture;

struct StateIdle : public State<CiApp>
{
    GET_SINGLETON_IMPL(StateIdle);

    void enter(CiApp* host)
    {
        timeline().apply(&mGlobalAlpha, 1.0f, 4.0f);
        console() << "StateIdle: " << mCurrentAnim << endl;
    }

    void update(CiApp* host);
};

State<CiApp>::Ref sFadeOutNextState;

struct StateFadeOut : public State<CiApp>
{
    GET_SINGLETON_IMPL(StateFadeOut);

    void enter(CiApp* host);

    void update(CiApp* host);
};

struct StateInteractive : public State<CiApp>
{
    GET_SINGLETON_IMPL(StateInteractive);

    void enter(CiApp* host)
    {
        timeline().apply(&mGlobalAlpha, 1.0f, 2.0f);
        console() << "StateInteractive: " << mCurrentAnim << endl;
    }

    void update(CiApp* host);

    void exit(CiApp* host)
    {
        //mCurrentAnim = -1;
        mGlobalAlpha = MIN_GLOBAL_ALPHA;
    }
};

bool mIsAlive = true;

struct CiApp : public AppBasic, StateMachine<CiApp>
{
    Config mConfigs[Config::kCount];
    int mConfigIds[kHourCount];

    CiApp() : StateMachine<CiApp>(this)
    {
        for (int i=0; i<kHourCount; i++)
        {
            if (i > 1 && i < 10)
            {
                mConfigIds[i] = -1;
            }
            else
            {
                mConfigIds[i] = 0;
            }
        }
    }

    void readProgramSettings()
    {
        fs::path configPath = getAssetPath("") / kProgSettingFileName;
        try
        {
            XmlTree tree(loadFile(configPath));
            for (int i=0; i<Config::kCount; i++)
            {
                mConfigs[i] = tree.getChild(toString(i)).getValue<Config>();
            }

            string str = tree.getChild("ids").getValue();
            for (int i=0; i<kHourCount; i++)
            {
                mConfigIds[i] = tree.getChild("ids").getChild(toString(i)).getValue<int>();
            }
        }
        catch (exception& e)
        {
            writeProgramSettings();
        }
    }

    void writeProgramSettings()
    {
        XmlTree tree = XmlTree::createDoc();
        for (int i=0; i<Config::kCount; i++)
        {
            XmlTree item(toString(i), toString(mConfigs[i]));
            tree.push_back(item);
        }
        XmlTree ids("ids", "");
        for (int i=0; i<kHourCount; i++)
        {
            ids.push_back(XmlTree(toString(i), toString(mConfigIds[i])));
        }
        tree.push_back(ids);

        fs::path configPath = getAssetPath("") / kProgSettingFileName;
        tree.write(writeFile(configPath));
    }

    void prepareSettings(Settings *settings)
    {
        readConfig();
        readProgramSettings();

        settings->setWindowPos(0, 0);
        settings->setWindowSize(Display::getMainDisplay()->getWidth(), Display::getMainDisplay()->getHeight());
    }

    void setup()
    {
        mMainGUI = params::InterfaceGl("params - press F1 to hide", Vec2i(300, getWindowHeight()));
        setupConfigUI(&mMainGUI);
        mMainGUI.setPosition(Vec2i(10, 100));

        for (int i=0; i<24; i++)
        {
            mGestures[i] = GestureDetector(-Vec3f::zAxis());
        }

        mHour = -1;
        mProbeConfig = -1;
        mCurrentConfig = NULL;

        mCurrentAnim = -1;

        for (int id=0; id<2; id++)
        {
            // parse "/assets/anim"
            // parse "/assets/anim_wall"
            const char* kAnimFolderNames[] = 
            {
                "anim",
                "anim_wall"
            };
            fs::path root = getAssetPath(kAnimFolderNames[id]);
            for (fs::directory_iterator it(root); it != kEndIt; ++it)
            {
                try
                {
                    qtime::MovieSurface anim = qtime::MovieSurface(it->path());
                    anim.setLoop(false);
                    mIdleAnims[id].push_back(anim);
                }
                catch (const exception& e)
                {
                    console() << e.what() << endl;
                }
            }
            mIdleChannels[id] = Channel(mIdleAnims[id][0].getWidth(), mIdleAnims[id][0].getHeight());

            const char* kKinectAnimFiles[] = 
            {
                "aniGlobe11", "aniWall11",
                "aniGlobe12", "aniWall12",
                "aniGlobe13", "aniWall13",
            };

            for (int k=0; k<kKinectAnimTypeCount; k++)
            {
                BOOST_FOREACH(string file, am::files(kKinectAnimFiles[k*2 + id]))
                {
                    try
                    {
                        Channel surf = loadImage(file);
                        mKinectAnimSequences[k].seqs[id].push_back(surf);
                    }
                    catch (...)
                    {
                    }
                }
            }
            mKinectSurfs[id] = Channel(mKinectAnimSequences[0].seqs[id][0].getWidth(), mKinectAnimSequences[0].seqs[id][0].getHeight());
        }

        mMainGUI.addParam("ANIMATION", &ANIMATION, "", true);
        {
            mMainGUI.addSeparator();
            vector<string> names;
            for (int i=0; i<Config::kCount; i++)
            {
                names.push_back("cfg# " + toString(i));
            }

            ADD_ENUM_TO_INT(mMainGUI, PROBE_CONFIG, names);
        }

        mMainGUI.addParam("current_hour", &mHour, "", true);
        mMainGUI.addText("Valid config are 0/1/2/3/4/5");
        mMainGUI.addText("And -1 means no config in this hour");
        for (int i=10; i<26; i++)
        {
            mMainGUI.addParam("hour# " + toString(i % kHourCount), &mConfigIds[i % kHourCount], "min=-1 max=5");
        }

        // osc setup
        mPadListener.setup(kOscPort);
        mPadListener.registerMessageReceived(this, &CiApp::onOscPadMessage);

        mKinectListener.setup(kKinectPort);
        mKinectListener.registerMessageReceived(this, &CiApp::onOscKinectMessage);

        changeToState(StateIdle::getSingleton());
    }

    void shutdown()
    {
        mIsAlive = false;
        writeProgramSettings();
    }

    void onOscKinectMessage(const osc::Message* msg)
    {
        if (!mIsAlive) return;

        if (mCurrentConfig == NULL) return;

        const AnimConfig& kinectCfg = mCurrentConfig->animConfigs[AnimConfig::kKinect];
        if (kinectCfg.loopCount == 0) return;

        const string& addr = msg->getAddress();
        if (addr != "/kinect") return;

        int PLAYER_ID = msg->getArgAsInt32(1);

        const int SHOULDER_CENTER = 4;
        const int HAND_LEFT = 9;
        const int HAND_RIGHT = 13;
        const int ids[3] = {SHOULDER_CENTER, HAND_LEFT, HAND_RIGHT};
        Vec3f poses[3];

        for (int i=0; i<3; i++)
        {
            string str = msg->getArgAsString(ids[i]);
            vector<string> xyzw = ci::split(str, ",");
            poses[i].set(fromString<float>(xyzw[0]), fromString<float>(xyzw[1]), fromString<float>(xyzw[2]));

            if (poses[0].z > KINECT_FAR || poses[0].z < KINECT_NEAR)
            {
                return;
            }
        }

        mLastKinectMsgSeconds = getElapsedSeconds();

        if (mCurrentState == StateIdle::getSingleton())
        {
            sFadeOutNextState = StateInteractive::getSingleton();
            changeToState(StateFadeOut::getSingleton());
        }

        if (mCurrentState != StateInteractive::getSingleton())
        {
            return;
        }

        static float sLastHitSeconds = getElapsedSeconds();
        if (getElapsedSeconds() - sLastHitSeconds < 2.0f)
        {
            return;
        }

        float wavingSpeed = 0;
        bool isDetected = false;
        for (int i=0; i<2; i++)
        {
            mGestures[PLAYER_ID*2+i].update(poses[0], poses[i + 1]);
            if (mGestures[PLAYER_ID*2+i].isDetected(KINECT_DISTANCE, &wavingSpeed))
            {
                isDetected = true;
            }
        }

        if (!isDetected)
        {
            return;
        }

        sLastHitSeconds = getElapsedSeconds();

        mKinectAnims.push_back(KinectAnim(wavingSpeed));
        console() << "Hit " << " speed " << wavingSpeed << endl;
    }

    void onOscPadMessage(const osc::Message* msg)
    {
        const string& addr = msg->getAddress();

        static bool sIsFirst = true;
        if (sIsFirst)
        {
            console() << "Remote IP: " << msg->getRemoteIp() << endl;
            sIsFirst = false;
            mPadSender.setup(msg->getRemoteIp(), kPadPort);
        }

        if (addr == "/ACK")
        {
            osc::Message msg;
            msg.setAddress("/msgBox");
            msg.addStringArg("Message received.");
            mPadSender.sendMessage(msg);
        }

        if (addr == "/schedule")
        {
            int hour = msg->getArgAsInt32(0);
            int prog = msg->getArgAsInt32(1);
            if (hour >=0 && hour < kHourCount 
                && prog >= -1 && prog < Config::kCount)
            {
                if (mCurrentHour == hour && mConfigIds[hour] != prog)
                {
                    // invalidate current hour, refer to updateProgram()
                    mHour = -1;
                }
                mConfigIds[hour] = prog;
            }
            writeProgramSettings();
            return;
        }

        if (addr == "/anim")
        {
            int idx = 0;
            const int cfg = msg->getArgAsInt32(idx++);
            Config& config = mConfigs[cfg];
            for (int i=0; i<AnimConfig::kCount; i++)
            {
                AnimConfig& animConfig = config.animConfigs[i];
                int loopCount = msg->getArgAsInt32(idx++);
                animConfig.loopCount = loopCount;
                animConfig.lightValue = msg->getArgAsFloat(idx++);
                animConfig.lightValue2 = msg->getArgAsFloat(idx++);
            }
            writeProgramSettings();
            return;
        }

        if (addr == "/WORLD_VISIBLE")
        {
            WORLD_VISIBLE = msg->getArgAsInt32(0);
        }
    }

    void keyUp(KeyEvent event)
    {
        switch (event.getCode())
        {
        case KeyEvent::KEY_ESCAPE:
            {
                quit();
                break;
            }
        case KeyEvent::KEY_F1:
            {
                GUI_VISIBLE = !GUI_VISIBLE;
                break;
            }
        }
    }

    void updateProgram()
    {
        mCurrentHour = getHour();

        // current program
        if (mHour != mCurrentHour)
        {
            mHour = mCurrentHour;
            //ANIMATION = 0;
            //mCurrentAnim = -1;
            if (mConfigIds[mHour] != -1)
            {
                int progId = constrain(mConfigIds[mHour], 0, Config::kCount - 1);
                mCurrentConfig = &mConfigs[progId];
                mElapsedLoopCount = mCurrentConfig->animConfigs[ANIMATION].loopCount - 1;
                //timeline().apply(&mGlobalAlpha, 0.0f, 1.0f);
                //timeline().appendTo(&mGlobalAlpha, 1.0f, 1.0f).delay(1.0f);
            }
            else
            {
                mCurrentConfig = NULL;
            }
        }

        // probe
        if (mProbeConfig != PROBE_CONFIG)
        {
            mProbeConfig = PROBE_CONFIG;
            mProgramGUI = params::InterfaceGl("cfg# " + toString(mProbeConfig), Vec2i(300, getWindowHeight()));
            mProgramGUI.setPosition(Vec2i(1058, 10));

            Config& prog = mConfigs[mProbeConfig];
            mProgramGUI.addSeparator();
            for (int i=0; i<AnimConfig::kCount; i++)
            {
                if (i == AnimConfig::kKinect)
                {
                    mProgramGUI.addText("Kinect");
                }
                else
                {
                    mProgramGUI.addText("Anim# " + toString(i));
                }
                mProgramGUI.addParam("loopCount of # " + toString(i), &prog.animConfigs[i].loopCount, "min=0");
                mProgramGUI.addParam("lightValue of # " + toString(i), &prog.animConfigs[i].lightValue, "min=0 max=1 step=0.01");
                mProgramGUI.addParam("lightValue2 of # " + toString(i), &prog.animConfigs[i].lightValue2, "min=0  max=1 step=0.01");
            }
        }
    }

    void update()
    {
        mRandomColorIndex += RANDOM_COLOR_SPEED;

        updateProgram();
        if (mCurrentConfig == NULL)
        {
            // TODO: black texture
            return;
        }

        updateSM();

        if (mCurrentAnim == -1) return;

        Channel* pGlobeChannel = NULL;
        Channel* pWallChannel = NULL;

        if (mCurrentState == StateInteractive::getSingleton())
        {
            mLedColor = mCurrentConfig->animConfigs[AnimConfig::kKinect].getColor();

            pGlobeChannel = &mKinectSurfs[0];
            pWallChannel = &mKinectSurfs[1];
        }
        else
        {
            mLedColor = mCurrentConfig->animConfigs[mCurrentAnim].getColor();

            pGlobeChannel = &mIdleChannels[0];
            pWallChannel = &mIdleChannels[1];
        }
        updateTextureFrom(mGlobeTexture, *pGlobeChannel);
        updateTextureFrom(mWallTexture, *pWallChannel);
    }

    void drawLedMapping()
    {
        gl::color(Color(mLedColor * mGlobalAlpha));
        gl::draw(mGlobeTexture, Vec2f(GLOBE_X, GLOBE_Y));
        gl::draw(mWallTexture, Vec2f(WALL_X, WALL_Y));
    }

    void draw()
    {
        gl::clear(ColorA::black());

        if (WORLD_VISIBLE == 0)
        {
            gl::setMatricesWindow(getWindowSize());
            drawLedMapping();
        }
        else if (WORLD_VISIBLE == 2)
        {
            gl::color(FULL_COLOR);
            gl::setMatricesWindow(getWindowSize());
            gl::drawSolidRect(Rectf(GLOBE_X, GLOBE_Y, GLOBE_X + kGlobePhysicsSize.x, GLOBE_Y + kGlobePhysicsSize.y + 1));
            gl::drawSolidRect(Rectf(WALL_X, WALL_Y, WALL_X + kWallPhysicsSize.x, WALL_Y + kWallPhysicsSize.y + 1));
        }

        if (GUI_VISIBLE)
        {
            mMainGUI.draw();
            mProgramGUI.draw();
        }
    }
};

void StateIdle::update(CiApp* host)
{
    float duration = 60;
    if (mCurrentAnim != -1)
    {
        for (int id=0; id<2; id++)
        {
            if (mIdleAnims[id][mCurrentAnim].checkNewFrame())
            {
                mIdleChannels[id] = mIdleAnims[id][mCurrentAnim].getSurface().getChannelRed();
            }
        }

        mCurrentMovieTime = mIdleAnims[0][mCurrentAnim].getCurrentTime();
        duration = mIdleAnims[0][mCurrentAnim].getDuration();
    }

    if (DEBUG_MODE)
    {
        DEBUG_ANIM = constrain(DEBUG_ANIM, 0, AnimConfig::kKinect);
        ANIMATION = DEBUG_ANIM;
        mElapsedLoopCount = 0;

        if (mCurrentMovieTime >= duration)
        {
            mCurrentAnim = -1;  // manually invalidate
        }
    }
    else
    {
        while (mCurrentMovieTime >= duration || mElapsedLoopCount >= mCurrentConfig->animConfigs[ANIMATION].loopCount)
        {
            mCurrentMovieTime = 0; // to make the first condition of while loop fail
            if (mElapsedLoopCount < mCurrentConfig->animConfigs[ANIMATION].loopCount)
            {
                mCurrentAnim = -1;  // manually invalidate
                mElapsedLoopCount++;
            }
            else
            {
                ANIMATION = (ANIMATION + 1) % mIdleAnims[0].size();
                mElapsedLoopCount = 0;
            }
        }
    }

    if (mCurrentAnim != ANIMATION)
    {
        for (size_t id=0; id<2; id++)
        {
            if (mCurrentAnim != -1)
            {
                mIdleAnims[id][mCurrentAnim].stop();
            }

            mIdleAnims[id][ANIMATION].seekToStart();
            mIdleAnims[id][ANIMATION].play();
            ip::fill(&mIdleChannels[id], (uint8_t)0);
        }
        mCurrentAnim = ANIMATION;
    }
}

void StateFadeOut::enter(CiApp* host)
{
    console() << "StateFadeOut: " << mCurrentAnim << endl;
    timeline().apply(&mGlobalAlpha, MIN_GLOBAL_ALPHA, FADE_OUT_SECONDS);
}

void StateFadeOut::update(CiApp* host)
{
    if (mGlobalAlpha <= MIN_GLOBAL_ALPHA)
    {
        // TODO: finishFn
        host->changeToState(sFadeOutNextState);
    }

    if (getElapsedSeconds() - mLastKinectMsgSeconds > 0.5f) // TODO
    {
        host->changeToState(StateIdle::getSingleton());
    }

    StateIdle::getSingleton()->update(host); // HACK
}

void StateInteractive::update(CiApp* host)
{
    int aliveMovieCount = 0;

    for (int id=0; id<2; id++)
    {
        ip::fill(&mKinectSurfs[id], (uint8_t)0);
    }

    mKinectAnims.remove_if(tr1::mem_fn(&KinectAnim::isFinished));

    BOOST_FOREACH(KinectAnim& anim, mKinectAnims)
    {
        aliveMovieCount++;

        Channel surfs[2];
        anim.get(&surfs[0], &surfs[1]);

        for (int id=0; id<2; id++)
        {
            KinectAnimSeq& kinectSeq = *anim.kinectSeq;

            for (int y=0; y<kinectSeq.seqs[id][0].getHeight(); y++)
            {
                for (int x=0; x<kinectSeq.seqs[id][0].getWidth(); x++)
                {
                    const uint8_t* pSrcRGB = surfs[id].getData(Vec2i(x, y));
                    uint8_t* pDstRGB = mKinectSurfs[id].getData(Vec2i(x, y));
                    pDstRGB[0] = min<int>((int)pDstRGB[0] + pSrcRGB[0], 255);
                }
            }
        }
    }

    if (aliveMovieCount == 0)
    {
        if (getElapsedSeconds() - mLastKinectMsgSeconds > KINECT_OUTOF_SECONDS)
        {
            host->changeToState(StateIdle::getSingleton());
        }
    }
}

CINDER_APP_BASIC(CiApp, RendererGl)
