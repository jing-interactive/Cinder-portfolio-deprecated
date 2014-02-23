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

#include "cinder/tuio/TuioClient.h"
#include "../../../_common/MiniConfig.h"
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include "../../../_common/AssetManager.h"
#include "../../../_common/SequenceAnim.h"

#define ASIO_DISABLE_BOOST_REGEX 0
#include "../../../_common/asio/asio.hpp"

#pragma warning(disable: 4244)

using namespace ci;
using namespace ci::app;
using namespace std;

const float kCamFov = 60.0f;
const int kTuioKinectPort = 3333;
const int kOscPadPort = 4444;
const float kLedOffset = 225.0f;
const int kThreadCount = 10;

fs::directory_iterator kEndIt;

size_t gIdCount = 0;
struct Led
{
    Led(const Vec3f& aPos, float aValue = 1.0f): pos(aPos), value(aValue), id(gIdCount++){}
    Vec3f pos;
    float value;
    size_t id;
};

typedef shared_ptr<SequenceAnimGray> AnimPtr;

static int getHour()
{
    SYSTEMTIME wtm;
    ::GetLocalTime(&wtm);
    return wtm.wHour;
}

// TODO: proto-buf?
struct AnimConfig
{
    static const int kCount = 10;

    AnimConfig()
    {
        lightValue = 1000;
        lightValue2 = 0;
        loopCount = 1;
    }

    Color getColor() const
    {
        // TODO: correct the mapping
        if (lightValue2 != 0)
        {
            return Color(randFloat(), 1, 1);
        }
        else
        {
            return Color(1, 1, 1);
        }
    }
    int lightValue;
    int lightValue2; // if non-zero, then random light value from (lightValue, lightValue2)
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
    Config()
    {
        isKinectEnabled = false;
    }

    AnimConfig animConfigs[AnimConfig::kCount];
    bool isKinectEnabled;

    friend ostream& operator<<(ostream& lhs, const Config& rhs)
    {
        for (int i=0; i<AnimConfig::kCount; i++)
        {
            lhs << rhs.animConfigs[i] << " ";
        }
        lhs << rhs.isKinectEnabled;
        return lhs;
    }

    friend istream& operator>>(istream& lhs, Config& rhs)
    {
        for (int i=0; i<AnimConfig::kCount; i++)
        {
            lhs >> rhs.animConfigs[i] >> std::ws;
        }
        lhs >> rhs.isKinectEnabled;
        return lhs;
    }
};

const string kProgSettingFileName = "ProgramSettings.xml";
const int kHourCount = 24; // valid hours for G9 are [10~23; 00; 01]

struct CiApp : public AppBasic
{
    Config mConfigs[Config::kCount];
    int mConfigIds[kHourCount];

    CiApp(): mWork(mIoService)
    {
        for (int i=0; i<kHourCount; i++)
        {
            if (i > 1 && i < 10)
            {
                mConfigIds[i] = -1;
            }
            else
            {
                mConfigIds[i] = rand() % Config::kCount;
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
        mFrameDelta = 0;
        mProbeProgram = -1;

        mCurrentCamDistance = -1;

        for (int i = 0; i < kThreadCount; ++i)
        {
            mThreads.create_thread(bind(&asio::io_service::run, &mIoService));
        }

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
                if (fs::is_directory(*it))
                {
                    AnimPtr anim = boost::make_shared<SequenceAnimGray>();
                    anim->setOneshot(true);
                    if (!loadAnimFromDir(*it, anim))
                        continue;
                    mAnims[id].push_back(anim);
                }
            }

            if (id == 0 && !mAnims[id].empty())
            {
                vector<string> animNames;
                for (size_t i=0; i<mAnims[id].size(); i++)
                {
                    animNames.push_back(mAnims[id][i]->name);
                }
                ADD_ENUM_TO_INT(mParams, ANIMATION, animNames);
            }

            {
                mParams.addSeparator();
                vector<string> names;
                for (int i=0; i<Config::kCount; i++)
                {
                    names.push_back("program# " + toString(i));
                }

                ADD_ENUM_TO_INT(mParams, PROBE_PROGRAM, names);
            }

            mParams.addSeparator();
            mParams.addParam("current_hour", &mHour, "", true);
            mParams.addText("Valid programs are 0/1/2/3/4/5");
            mParams.addText("And -1 means no program in this hour");
            for (int i=10; i<26; i++)
            {
                mParams.addParam("hour# " + toString(i % kHourCount), &mConfigIds[i % kHourCount], "min=-1 max=5");
            }
        }

        // osc setup
        mPadListener.setup(kOscPadPort);
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
        mIoService.stop();
        try
        {
            mThreads.join_all();
        }
        catch (...)
        {

        }
    }

    void onOscMessage(const osc::Message* msg)
    {
        const string& addr = msg->getAddress();

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
            int cfg = msg->getArgAsInt32(0);
            Config& config = mConfigs[cfg];
            config.isKinectEnabled = msg->getArgAsInt32(1);
            for (int i=0; i<AnimConfig::kCount; i++)
            {
                AnimConfig& animConfig = config.animConfigs[i];
                bool isEnabled = msg->getArgAsInt32(1 + 4*i);
                animConfig.loopCount = msg->getArgAsInt32(2 + 4*i);
                if (!isEnabled)
                {
                    animConfig.loopCount = 0;
                }
                animConfig.lightValue = msg->getArgAsInt32(3 + 4*i);
                animConfig.lightValue2 = msg->getArgAsInt32(4 + 4*i);
            }
            return;
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
                mCurrentProgram = &mConfigs[progId];
                mRemainingLoopForAnim = mCurrentProgram->animConfigs[ANIMATION].loopCount;
            }
            else
            {
                mCurrentProgram = NULL;
            }
        }

        // probe
        if (mProbeProgram != PROBE_PROGRAM)
        {
            mProbeProgram = PROBE_PROGRAM;
            mProgramGUI = params::InterfaceGl("program# " + toString(mProbeProgram), Vec2i(300, getWindowHeight()));
            Config& prog = mConfigs[mProbeProgram];
            mProgramGUI.addParam("isKinectEnabled", &prog.isKinectEnabled);
            mProgramGUI.addSeparator();
            for (int i=0; i<AnimConfig::kCount; i++)
            {
                mProgramGUI.addText("anim# " + toString(i+1));
                mProgramGUI.addParam("loopCount of # " + toString(i+1), &prog.animConfigs[i].loopCount, "min=0");
                mProgramGUI.addParam("lightValue of # " + toString(i+1), &prog.animConfigs[i].lightValue, "min=0");
                mProgramGUI.addParam("lightValue2 of # " + toString(i+1), &prog.animConfigs[i].lightValue2, "min=0");
            }
        }
    }

    void updateAnim()
    {
        // calculate ANIMATION
        if (!mAnims[0][ANIMATION]->isAlive() || 
            (mRemainingLoopForAnim < 1 && mAnims[0][ANIMATION]->isAlive()))
        {
            if (mRemainingLoopForAnim > 0)
            {
                mCurrentAnim = -1;  // manually invalidate
                mRemainingLoopForAnim--;
            }
            else
            {
                ANIMATION = (ANIMATION + 1) % AnimConfig::kCount;
                mRemainingLoopForAnim = mCurrentProgram->animConfigs[ANIMATION].loopCount;
            }
        }

        if (mCurrentAnim != ANIMATION)
        {
            mCurrentAnim = ANIMATION;
            mLedColor = mCurrentProgram->animConfigs[mCurrentAnim].getColor();
            for (size_t i=0; i<2; i++)
            {
                mAnims[i][mCurrentAnim]->reset();
            }
        }

        for (size_t i=0; i<2; i++)
        {
            mAnims[i][mCurrentAnim]->update(mFrameDelta * ANIM_SPEED);
        }
    }

    void update()
    {
        static float sPrevSec = getElapsedSeconds();

        mIoService.poll();

        updateProgram();
        if (mCurrentProgram == NULL)
        {
            BOOST_FOREACH(Led& led, mLeds)
            {
                led.value = 0; // TODO: black color?
            }
            return;
        }

        vector<tuio::Cursor> cursors;
        if (mCurrentProgram->isKinectEnabled)
        {
            cursors = mTuioClient.getCursors();
        }

        const Channel* pChannel = NULL;
        if (cursors.size() == 2)
        {
            ip::fill(&mKinectChan, (uint8_t)0);
            for (int i=0; i<2; i++)
            {
                Vec2f pos = cursors[i].getPos();
                int width = (1.0f - pos.y) * mKinectChan.getWidth();
                int halfH = mKinectChan.getHeight(); // TODO: global & cache
                for (int x=0; x<width; x++)
                {
                    for (int y=halfH*i; y<(halfH*(i+1)); y++)
                    {
                        *mKinectChan.getData(x, y) = 122; // TODO: which value?
                    }
                }
                // TODO: getSpeed()?
                pChannel = &mKinectChan;
            }
        }
        else
        {
            updateAnim();
            pChannel = &mAnims[0][mCurrentAnim]->getFrame();
        }

        int32_t width = pChannel->getWidth();
        int32_t height = pChannel->getHeight();

        float kW = pChannel->getWidth() / 1029.0f;
        float kH = pChannel->getHeight() / 124.0f;
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
            uint8_t value = *pChannel->getData(Vec2i(kW * cx, kH * cy));
            led.value = value / 255.f;
        }

        if (mCurrentCamDistance != CAM_DISTANCE)
        {
            mCurrentCamDistance = CAM_DISTANCE;
            mCamera.setPerspective(kCamFov, getWindowAspectRatio(), 0.1f, 1000.0f);
            mCamera.lookAt(Vec3f(- mAABB.getMax().x * mCurrentCamDistance, mAABB.getMax().y * 0.5f, 0.0f), Vec3f::zero());
        }

        mFrameDelta = getElapsedSeconds() - sPrevSec;
        sPrevSec = getElapsedSeconds();
    }

    void drawLedMapping()
    {
        // TODO
        BOOST_FOREACH(Led& led, mLeds)
        {

        }
    }

    void draw()
    {
        gl::enableDepthRead();
        gl::enableDepthWrite();

        gl::clear(ColorA::gray(43 / 255.f));
        gl::setMatrices(mCamera);

        if (mCurrentProgram == NULL)
        {
            if (GUI_VISIBLE)
            {
                mParams.draw();
                mProgramGUI.draw();
            }
            return;
        }

        float kSceneOffsetY = 0;//SCENE_OFFSET_Y * REAL_TO_VIRTUAL;

        if (COORD_FRAME_VISIBLE)
        {
            gl::pushModelView();
            gl::translate(0, mAABB.getSize().y * -0.5f, kSceneOffsetY);
            gl::rotate(CAM_ROTATION);
            gl::scale(50, 50, 50);
            gl::drawCoordinateFrame();
            gl::popModelView();
        }

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
                gl::Texture tex = mAnims[1][mCurrentAnim]->getTexture();
                tex.enableAndBind();
                gl::draw(mVboWall);
                tex.disable();
            }
        }
        gl::popModelView();

        // 2D
        gl::setMatricesWindow(getWindowSize());
        if (ANIM_COUNT_VISIBLE && mCurrentAnim != -1)
        {
            gl::drawString(toString(mAnims[0][mCurrentAnim]->index), Vec2f(10, 10));
        }
        drawLedMapping();

        if (REFERENCE_VISIBLE && mCurrentAnim != -1)
        {
            const float kOffY = REFERENCE_OFFSET_Y;
            const Rectf kRefGlobeArea(28, 687 + kOffY, 28 + 636, 687 + 90 + kOffY);
            const Rectf kRefWallArea(689, 631 + kOffY, 689 + 84, 631 + 209 + kOffY);

            gl::draw(mAnims[0][mCurrentAnim]->getTexture(), kRefGlobeArea);
            gl::draw(mAnims[1][mCurrentAnim]->getTexture(), kRefWallArea);
        }

        if (GUI_VISIBLE)
        {
            mParams.draw();
            mProgramGUI.draw();
        }
    }

    void safeLoadImage(fs::path imagePath, AnimPtr anim, size_t index)
    {
        try
        {
            anim->frames[index] = loadImage(imagePath);
        }
        catch (exception& e)
        {
#ifdef _DEBUG
            console() << e.what() << endl;
#endif
        }
#ifdef _DEBUG
        //console() << imagePath << endl;
#endif
    }

    bool loadAnimFromDir(fs::path dir, AnimPtr anim) 
    {
        double startTime = getElapsedSeconds();

        anim->name = dir.filename().string();

        int fileCount = distance(fs::directory_iterator(dir), kEndIt);
        anim->frames.resize(fileCount);

        size_t index = 0;
        for (fs::directory_iterator it(dir); it != kEndIt; ++it, ++index)
        {
            if (!fs::is_regular_file(*it))
                continue;

            mIoService.post(boost::bind(&CiApp::safeLoadImage, this, *it, anim, index));
        }

        if (anim->frames.empty())
        {
            return false;
        }

        console() << dir << ": " << getElapsedSeconds() - startTime << endl;

        return true;
    }

private:
    params::InterfaceGl mParams;
    params::InterfaceGl mProgramGUI;

    osc::Listener   mPadListener;
    tuio::Client    mTuioClient;
    float           mFrameDelta;
    Channel         mKinectChan;

    vector<Led>     mLeds;
    int             mCurrentCamDistance;
    AxisAlignedBox3f mAABB;
    CameraPersp     mCamera;

    vector<AnimPtr>    mAnims[2];

    int             mCurrentAnim;

    gl::VboMesh     mVboWall;

    asio::io_service mIoService;
    asio::io_service::work mWork;
    boost::thread_group mThreads;

    int             mProbeProgram;
    Config*  mCurrentProgram;

    Color           mLedColor;
    int             mHour;
    int             mRemainingLoopForAnim;
};

CINDER_APP_BASIC(CiApp, RendererGl)
