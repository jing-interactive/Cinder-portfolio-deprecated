#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/Xml.h"
#include "cinder/CinderMath.h"
#include "cinder/Rand.h"
#include "cinder/ip/Fill.h"

#include "cinder/gl/Fbo.h"
#include "cinder/params/Params.h"

#include "cinder/Utilities.h"
#include "cinder/Thread.h"

#include "cinder/qtime/QuickTime.h"

#include "cinder/tuio/TuioClient.h"
#include "cinder/osc/OscSender.h"

#include "../../../_common/MiniConfig.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include "../../../_common/AssetManager.h"

#pragma warning(disable: 4244)

using namespace ci;
using namespace ci::app;
using namespace std;

const float kCamFov = 60.0f;
const int kTuioKinectPort = 3333;
const int kOscPort = 4444;
const int kPadPort = 5555;
const float kLedOffset = 225.0f;
const int kThreadCount = 10;
const Vec2i kGlobePhysicsSize(257, 18);
const Vec2i kWallPhysicsPos(kGlobePhysicsSize.x, 0);
const Vec2i kWallPhysicsSize(21, 56);

fs::directory_iterator kEndIt;

static void updateTextureFromSurface(gl::Texture& tex, const Surface& surf)
{
    if (tex && surf.getSize() == tex.getSize())
        tex.update(surf);
    else
        tex = gl::Texture(surf);
}

size_t gIdCount = 0;
struct Led
{
    Led(const Vec3f& aPos, float aValue = 1.0f): pos(aPos), value(aValue), id(gIdCount++){}
    Vec3f pos;
    Vec2i pos2d;
    float value;
    size_t id;
};

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
        float value = lightValue;
        // TODO: correct the mapping
        if (lightValue2 != 0)
        {
            if (lightValue2 > lightValue)
            {
                value = randFloat(lightValue, lightValue2);
            }
            else
            {
                value = randFloat(lightValue2, lightValue);
            }
        }
        return Color(value, 0.0f, 1.0f - value);
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

struct CiApp : public AppBasic
{
    Config mConfigs[Config::kCount];
    int mConfigIds[kHourCount];

    CiApp()
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
        settings->setWindowSize(800, 800);
    }

    void setup()
    {
        mParams = params::InterfaceGl("params - press F1 to hide", Vec2i(300, getWindowHeight()));
        setupConfigUI(&mParams);

        mKinectChan = loadImage(getAssetPath("black-for-kinect.jpg"));
        mHour = -1;
        mProbeConfig = -1;

        mCurrentCamDistance = -1;

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
                if (fs::is_regular_file(*it))
                {
                    try
                    {
                        qtime::MovieSurface anim = qtime::MovieSurface(it->path());
                        anim.setLoop(false);
                        mAnims[id].push_back(anim);
                    }
                    catch (const exception& e)
                    {
                        console() << e.what() << endl;
                    }
                }
            }
        }

        mParams.addParam("ANIMATION", &ANIMATION, "", true);
        {
            mParams.addSeparator();
            vector<string> names;
            for (int i=0; i<Config::kCount; i++)
            {
                names.push_back("cfg# " + toString(i));
            }

            ADD_ENUM_TO_INT(mParams, PROBE_CONFIG, names);
        }

        mParams.addParam("current_hour", &mHour, "", true);
        mParams.addText("Valid config are 0/1/2/3/4/5");
        mParams.addText("And -1 means no config in this hour");
        for (int i=10; i<26; i++)
        {
            mParams.addParam("hour# " + toString(i % kHourCount), &mConfigIds[i % kHourCount], "min=-1 max=5");
        }

        // osc setup
        mPadListener.setup(kOscPort);
        mPadListener.registerMessageReceived(this, &CiApp::onOscMessage);

        mTuioClient.connect(kTuioKinectPort);

        // parse leds.txt
        ifstream ifs(getAssetPath("leds.txt").string().c_str());
        int id;
        float x, y, z;

        Vec3f maxBound = Vec3f::zero();
        Vec3f minBound = Vec3f(FLT_MAX, FLT_MAX, FLT_MAX);

        int ledRadius = kLedOffset * REAL_TO_VIRTUAL / 2;
        while (ifs >> id >> x >> z >> y)
        {
            //x -= (245 - ledRadius);
            Vec3f pos(x, y, z);
            pos *= REAL_TO_VIRTUAL;
            mLeds.push_back(Led(pos));

            minBound.x = min<float>(minBound.x, pos.x - ledRadius);
            minBound.y = min<float>(minBound.y, pos.y);
            minBound.z = min<float>(minBound.z, pos.z - ledRadius);

            maxBound.x = max<float>(maxBound.x, pos.x + ledRadius);
            maxBound.y = max<float>(maxBound.y, pos.y);
            maxBound.z = max<float>(maxBound.z, pos.z + ledRadius);
        }

        mAABB = AxisAlignedBox3f(minBound, maxBound);

        // wall
        {   
            gl::VboMesh::Layout layout;
            layout.setStaticTexCoords2d();
            layout.setStaticPositions();
            //layout.setStaticColorsRGB();

            const size_t kNumVertices = 4;
            vector<Vec3f> positions(kNumVertices);
            // CCW
            // #3: -271.0, 9748.0 ---- #2: 4129.0, 9748.0
            //
            // #1: -271.0, -1452.0 ---- #0: 4129.0, -1452.0 
            positions[0] = Vec3f(4129.0, -1452.0, 33626);
            positions[1] = Vec3f(-271.0, -1452.0, 33626);
            positions[2] = Vec3f(-271.0, 9748.0, 33626);
            positions[3] = Vec3f(4129.0, 9748.0, 33626);
            for (size_t i=0; i<kNumVertices; i++)
            {
                positions[i] *= REAL_TO_VIRTUAL;
            }

            vector<Vec2f> texCoords(kNumVertices);
            texCoords[0] = Vec2f(1, 1);
            texCoords[1] = Vec2f(0, 1);
            texCoords[2] = Vec2f(0, 0);
            texCoords[3] = Vec2f(1, 0);

            vector<Color> colors(kNumVertices);
            colors[0] = Color(1, 0, 0);
            colors[1] = Color(0, 1, 0);
            colors[2] = Color(0, 0, 1);
            colors[3] = Color(1, 1, 1);

            mVboWall = gl::VboMesh(kNumVertices, 0, layout, GL_QUADS);
            mVboWall.bufferPositions(positions);
            mVboWall.bufferTexCoords2d(0, texCoords);
            //mVboWall.bufferColorsRGB(colors);
        }
    }

    void shutdown()
    {
        writeProgramSettings();
    }

    void onOscMessage(const osc::Message* msg)
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
                mConfigIds[hour] = prog;
            }
            return;
        }

        if (addr == "/anim")
        {
            int idx = 0;
            int cfg = msg->getArgAsInt32(idx++);
            Config& config = mConfigs[cfg];
            for (int i=0; i<AnimConfig::kCount; i++)
            {
                AnimConfig& animConfig = config.animConfigs[i];
                animConfig.loopCount = msg->getArgAsInt32(idx++);
                animConfig.lightValue = msg->getArgAsFloat(idx++);
                animConfig.lightValue2 = msg->getArgAsFloat(idx++);
            }
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
        // current program
        int hour = getHour();
        if (mHour != hour)
        {
            mHour = hour;
            ANIMATION = 0;
            mCurrentAnim = -1;
            if (mConfigIds[mHour] != -1)
            {
                int progId = constrain(mConfigIds[mHour], 0, Config::kCount - 1);
                mCurrentConfig = &mConfigs[progId];
                mRemainingLoopForAnim = mCurrentConfig->animConfigs[ANIMATION].loopCount;
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
                mProgramGUI.addParam("lightValue of # " + toString(i), &prog.animConfigs[i].lightValue, "min=0");
                mProgramGUI.addParam("lightValue2 of # " + toString(i), &prog.animConfigs[i].lightValue2, "min=0");
            }
        }
    }

    void updateAnim()
    {
        // calculate ANIMATION
        const float time = mAnims[0][ANIMATION].getCurrentTime();
        const float duration = mAnims[0][ANIMATION].getDuration();

        if (time > duration - FLT_EPSILON)
        {
            if (mRemainingLoopForAnim > 1)
            {
                mCurrentAnim = -1;  // manually invalidate
                mRemainingLoopForAnim--;
            }
            else
            {
                ANIMATION = (ANIMATION + 1) % AnimConfig::kKinect;
                mRemainingLoopForAnim = mCurrentConfig->animConfigs[ANIMATION].loopCount;
            }
        }

        if (mCurrentAnim != ANIMATION)
        {
            mCurrentAnim = ANIMATION;
            mLedColor = mCurrentConfig->animConfigs[mCurrentAnim].getColor();
            for (size_t i=0; i<2; i++)
            {
                mAnims[i][mCurrentAnim].seekToStart();
                mAnims[i][mCurrentAnim].play();
                while (!mAnims[i][mCurrentAnim].checkNewFrame())
                {
                    sleep(30);
                }
            }
        }
    }

    void update()
    {
        static float sPrevSec = getElapsedSeconds();

        updateProgram();
        if (mCurrentConfig == NULL)
        {
            BOOST_FOREACH(Led& led, mLeds)
            {
                led.value = 0; // TODO: black color?
            }
            return;
        }

        vector<tuio::Cursor> cursors;
        if (mCurrentConfig->animConfigs[AnimConfig::kKinect].loopCount > 0)
        {
            cursors = mTuioClient.getCursors();
        }

        const Surface* pSurface = NULL;
        if (cursors.size() == 2)
        {
            ip::fill(&mKinectChan, Color::black());
            for (int i=0; i<2; i++)
            {
                Vec2f pos = cursors[i].getPos();
                int width = (1.0f - pos.y) * mKinectChan.getWidth();
                int halfH = mKinectChan.getHeight(); // TODO: global & cache
                for (int x=0; x<width; x++)
                {
                    for (int y=halfH*i; y<(halfH*(i+1)); y++)
                    {
                        *mKinectChan.getData(Vec2i(x, y)) = 122; // TODO: which value?
                    }
                }
                // TODO: getSpeed()?
                pSurface = &mKinectChan;
            }
        }
        else
        {
            updateAnim();
            pSurface = &mAnims[0][mCurrentAnim].getSurface();
        }

        int32_t width = pSurface->getWidth();
        int32_t height = pSurface->getHeight();

        float kW = pSurface->getWidth() / 1029.0f;
        float kH = pSurface->getHeight() / 124.0f;
        BOOST_FOREACH(Led& led, mLeds)
        {
            // online solver
            // http://www.bluebit.gr/matrix-calculator/linear_equations.aspx

            //3321  1  103
            //32936 1  1023
            float cx = 0.031065338510890f * led.pos.z / REAL_TO_VIRTUAL - 0.167989194664881f;

            //245  1  2
            //4070 1  122
            float cy = 0.031372549019608f * led.pos.x / REAL_TO_VIRTUAL - 5.686274509803920f;

            Vec2i sample2D(kW * cx, kH * cy);
            uint8_t value = *pSurface->getData(sample2D);

            led.pos2d = sample2D * kGlobePhysicsSize / pSurface->getSize();
            led.value = value / 255.f;
        }

        if (mCurrentCamDistance != CAM_DISTANCE)
        {
            mCurrentCamDistance = CAM_DISTANCE;
            mCamera.setPerspective(kCamFov, getWindowAspectRatio(), 0.1f, 1000.0f);
            mCamera.lookAt(Vec3f(- mAABB.getMax().x * mCurrentCamDistance, mAABB.getMax().y * 0.5f, 0.0f), Vec3f::zero());
        }

        sPrevSec = getElapsedSeconds();
    }

    void drawLedMapping()
    {
        gl::enableAlphaBlending();
        BOOST_FOREACH(Led& led, mLeds)
        {
            gl::color(ColorA(mLedColor, led.value));
            gl::drawPoint(led.pos2d);
        }
        gl::disableAlphaBlending();

        gl::color(Color(mLedColor));

        if (mCurrentAnim != -1 && mAnims[1][mCurrentAnim].checkNewFrame())
        {
            // TODO: use pSurface
            updateTextureFromSurface(mWallTexture, mAnims[1][mCurrentAnim].getSurface());
        }
        gl::draw(mWallTexture, Rectf(kWallPhysicsPos, kWallPhysicsPos + kWallPhysicsSize));
    }

    void draw()
    {
        gl::enableDepthRead();
        gl::enableDepthWrite();

        gl::clear(ColorA::gray(43 / 255.f));

        if (REFERENCE_VISIBLE)
        {
            gl::setMatrices(mCamera);
            draw3D();

            gl::setMatricesWindow(getWindowSize());
            draw2D();
        }

        if (WORLD_VISIBLE)
        {
            gl::setMatricesWindow(getWindowSize());
            drawLedMapping();
        }

        if (GUI_VISIBLE)
        {
            mParams.draw();
            mProgramGUI.draw();
        }
    }

    void draw2D() 
    {
        if (mCurrentAnim != -1)
        {
            const float kOffY = REFERENCE_OFFSET_Y;
            const Rectf kRefGlobeArea(28, 687 + kOffY, 28 + 636, 687 + 90 + kOffY);
            const Rectf kRefWallArea(689, 631 + kOffY, 689 + 84, 631 + 209 + kOffY);

            updateTextureFromSurface(mGlobeTexture, mAnims[0][mCurrentAnim].getSurface());
            gl::draw(mGlobeTexture, kRefGlobeArea);
            gl::draw(mWallTexture, kRefWallArea);
        }
    }

    void draw3D() 
    {
        if (mCurrentConfig == NULL)
        {
            if (GUI_VISIBLE)
            {
                mParams.draw();
                mProgramGUI.draw();
            }
            return;
        }

        float kSceneOffsetY = 0;//SCENE_OFFSET_Y * REAL_TO_VIRTUAL;

        gl::pushModelView();
        {
            Vec3f trans = mAABB.getSize() * -0.5f;
            trans.x *= -1;
            trans.y += kSceneOffsetY;
            gl::rotate(CAM_ROTATION);
            gl::translate(trans);

            gl::scale(-1, 1, 1);

            // lines
            gl::enableAlphaBlending();
            if (LINES_VISIBLE)
            {
                gl::disableDepthWrite();
                gl::color(ColorA::gray(76 / 255.f, 76 / 255.f));
                BOOST_FOREACH(const Led& led, mLeds)
                {
                    gl::drawLine(led.pos, Vec3f(led.pos.x, CEILING_HEIGHT, led.pos.z));
                }
            }

            // spheres
            gl::enableDepthWrite();
            BOOST_FOREACH(const Led& led, mLeds)
            {
                gl::color(ColorA(mLedColor, constrain(led.value, SPHERE_MIN_ALPHA, 1.0f)));
                gl::drawSphere(led.pos, SPHERE_RADIUS);
            }
            gl::disableAlphaBlending();

            if (mCurrentAnim != -1)
            {
                // TODO: state??
                // wall
                updateTextureFromSurface(mWallTexture, mAnims[1][mCurrentAnim].getSurface());
                mWallTexture.enableAndBind();
                gl::draw(mVboWall);
                mWallTexture.disable();
            }
        }
        gl::popModelView();
    }

private:
    params::InterfaceGl mParams;
    params::InterfaceGl mProgramGUI;

    osc::Listener   mPadListener;
    osc::Sender     mPadSender;
    tuio::Client    mTuioClient;
    Surface         mKinectChan;

    vector<Led>     mLeds;
    int             mCurrentCamDistance;
    AxisAlignedBox3f mAABB;
    CameraPersp     mCamera;

    vector<qtime::MovieSurface>    mAnims[2];

    int             mCurrentAnim;
    int             ANIMATION;

    gl::VboMesh     mVboWall;

    int             mProbeConfig;
    Config*         mCurrentConfig;

    Color           mLedColor;
    int             mHour;
    int             mRemainingLoopForAnim;

    gl::Texture     mGlobeTexture, mWallTexture;
};

CINDER_APP_BASIC(CiApp, RendererGl)
