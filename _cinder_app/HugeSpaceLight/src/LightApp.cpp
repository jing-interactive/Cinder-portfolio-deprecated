#include "LightApp.h"
#include "States.h"
#include "Config.h"

#include "cinder/ImageIo.h"
#include "cinder/CinderMath.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"

#include <fstream>
#include <boost/foreach.hpp>
#include <list>

#include "../../../_common/AssetManager.h"

const Vec2i kGlobePhysicsSize(257, 18);
const Vec2i kWallPhysicsSize(21, 56);

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

params::InterfaceGl mMainGUI;
params::InterfaceGl mProgramGUI;


gl::Texture     mGlobeTexture, mWallTexture;

Color           mLedColor;

AnimSquence mKinectAnims[kKinectAnimCount];
AnimSquence mIdleAnims[kIdleAnimCount];

Channel         mKinectSurfs[2]; // sum of mKinectBullets

int             mCurrentAnim;
int             ANIMATION;

int             mProbeConfig;
struct Config*  mCurrentConfig;

bool mIsAlive = true;

LightApp::LightApp() : StateMachine<LightApp>(this)
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

void LightApp::prepareSettings(Settings *settings)
{
    readConfig();
    readProgramSettings();

    settings->setWindowPos(0, 0);
    settings->setWindowSize(Display::getMainDisplay()->getWidth(), Display::getMainDisplay()->getHeight());
}

void loadImages()
{
    char folderName[256];
    const char* kIdleFolders[] = 
    {
        "aniGlobe", 
        "aniWall",
    };

    Channel blankChannels[] = 
    {
        loadImage(getAssetPath("blankGlobe.jpg")),
        loadImage(getAssetPath("blankWall.jpg"))
    };

    const size_t kBlankFileSizes[] =
    {
        1049,
        717
    };

    for (int id=0; id<2; id++)
    {
        for (int k=0; k<kIdleAnimCount; k++)
        {
            sprintf(folderName, "%s%02d", kIdleFolders[id], 1+k);
            const vector<string>& files = am::files(folderName);
            for (int i=0; i<files.size(); i++ )
            {
                if (TARGET_FRAME_COUNT > 0 && i == TARGET_FRAME_COUNT) break;

                if (fs::file_size(files[i]) <= kBlankFileSizes[id])
                {
                    mIdleAnims[k].seqs[id].push_back(blankChannels[id]);
                }
                else
                {
                    mIdleAnims[k].seqs[id].push_back(loadImage(files[i]));
                }
            }
        }
        mIdleChannels[id] = Channel(mIdleAnims[0].seqs[id][0].getWidth(), mIdleAnims[0].seqs[id][0].getHeight());

        for (int k=0; k<kKinectAnimCount; k++)
        {
            sprintf(folderName, "%s%02d", kIdleFolders[id], 11+k);
            const vector<string>& files = am::files(folderName);
            for (int i=0; i<files.size(); i++ )
            {
                if (TARGET_FRAME_COUNT > 0 && i == TARGET_FRAME_COUNT) break;

                if (fs::file_size(files[i]) <= kBlankFileSizes[id])
                {
                    mKinectAnims[k].seqs[id].push_back(blankChannels[id]);
                }
                else
                {
                    mKinectAnims[k].seqs[id].push_back(loadImage(files[i]));
                }
            }
        }
        mKinectSurfs[id] = Channel(mKinectAnims[0].seqs[id][0].getWidth(), mKinectAnims[0].seqs[id][0].getHeight());
    }
}

void LightApp::setup()
{
    mGlobalAlpha = 1.0f;

    mMainGUI = params::InterfaceGl("params - press F1 to hide", Vec2i(300, getWindowHeight()));
    setupConfigUI(&mMainGUI);
    mMainGUI.setPosition(Vec2i(10, 100));

    mHour = -1;
    mProbeConfig = -1;
    mCurrentConfig = NULL;

    mCurrentAnim = -1;

    loadImages();

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

    setupOsc();

    changeToState(StateIdle::getSingleton());
}

void LightApp::shutdown()
{
    mIsAlive = false;
    writeProgramSettings();
}

void LightApp::keyUp(KeyEvent event)
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

void LightApp::update()
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

void LightApp::draw()
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

CINDER_APP_BASIC(LightApp, RendererGl)
