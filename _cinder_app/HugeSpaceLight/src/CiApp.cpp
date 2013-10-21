#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Arcball.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

#include "cinder/Utilities.h"
#include "cinder/osc/OscListener.h"
#include "../../../_common/MiniConfig.h"
#include <fstream>
#include <boost/foreach.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

const float kCamFov = 60.0f;
const int kOscPort = 3333;
const float kLedOffset = 245.0f;

fs::directory_iterator end_iter;

struct Led
{
    Led(const Vec3f& aPos, float aValue = 1.0f) :
        pos(aPos), value(aValue){}
    Vec3f pos;
    float value;
};

struct Anim
{
    Anim()
    {
        reset();
    }
    string name;
    float index;
    vector<Surface> frames;
    vector<gl::Texture> textures;

    void reset()
    {
        index = 0;
    }

    void update(float speed)
    {
        index += speed;
        if (index >= frames.size())
        {
            index = 0;
        }
    }

    const Surface& getFrame() const
    {
        return frames[static_cast<int>(index)];
    }

    const gl::Texture& getTexture()
    {
        if (textures.size() != frames.size())
        {
            textures.resize(frames.size());
        }
        int id = static_cast<int>(index);

        if (!textures[id])
        {
            textures[id] = gl::Texture(frames[id]);
        }
        return textures[id];
    }
};

struct CiApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        readConfig();
        
        settings->setWindowPos(0, 0);
        settings->setWindowSize(800, 600);
    }

    void setup()
    {
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));

        // parse "/assets/anim"
        fs::path root = getAssetPath("anim");
        for ( fs::directory_iterator dir_iter(root); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_directory(*dir_iter) )
            {
                Anim anim;
                if (!loadAnimFromDir(*dir_iter, anim))
                    continue;
                mAnims.push_back(anim);
            }
        }

        mCurrentAnim = 0;
        mNextAnim = 0;
        if (!mAnims.empty())
        {
            vector<string> animNames;
            for (int i=0; i<mAnims.size(); i++)
            {
                animNames.push_back(mAnims[i].name);
            }
            mParams.addParam("ANIMATION", animNames, &mNextAnim);
        }

        // parse "/assets/anim_wall"
        // TODO: merge
        root = getAssetPath("anim_wall");
        for ( fs::directory_iterator dir_iter(root); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_directory(*dir_iter) )
            {
                Anim anim;
                if (!loadAnimFromDir(*dir_iter, anim))
                    continue;
                mAnimWalls.push_back(anim);
            }
        }

        mCurrentAnimWall = 0;
        mNextAnimWall = 0;
        if (!mAnimWalls.empty())
        {
            vector<string> animNames;
            for (int i=0; i<mAnimWalls.size(); i++)
            {
                animNames.push_back(mAnimWalls[i].name);
            }
            mParams.addParam("ANIMATION_WALL", animNames, &mNextAnimWall);
        }

        // MiniConfig.xml
        setupConfigUI(&mParams);

        // osc setup
        mListener.setup(kOscPort);
        mListener.registerMessageReceived(this, &CiApp::onOscMessage);

        // parse leds.txt
        ifstream ifs(getAssetPath("leds.txt").string().c_str());
        int id;
        float x, y, z;

        Vec3f maxBound = Vec3f::zero();
        Vec3f minBound = Vec3f(FLT_MAX, FLT_MAX, FLT_MAX);

        int ledOffset = kLedOffset * SPHERE_UNIT_RATIO;
        while (ifs >> id >> x >> z >> y)
        {
            Vec3f pos(x, y, z);
            pos *= SPHERE_UNIT_RATIO;
            mLeds.push_back(Led(pos));

            minBound.x = min<float>(minBound.x, pos.x - ledOffset);
            minBound.y = min<float>(minBound.y, pos.y);
            minBound.z = min<float>(minBound.z, pos.z/* - ledOffset*/);

            maxBound.x = max<float>(maxBound.x, pos.x + ledOffset);
            maxBound.y = max<float>(maxBound.y, pos.y);
            maxBound.z = max<float>(maxBound.z, pos.z + ledOffset);
        }

        mAABB = AxisAlignedBox3f(minBound, maxBound);
        BOOST_FOREACH(Led& led, mLeds)
        {
            //led.pos -= minBound;
        }

        // camera setup
        CameraPersp initialCam;
        initialCam.setPerspective(kCamFov, getWindowAspectRatio(), 0.1f, 1000.0f);
        initialCam.lookAt(Vec3f(- maxBound.x * 5.0f, maxBound.y * 0.7f, - maxBound.z * 0.5f), Vec3f::zero());
        mMayaCam.setCurrentCam(initialCam);

        mPrevSec = getElapsedSeconds();

        // wall
        {
            gl::VboMesh::Layout layout;
            layout.setStaticTexCoords2d();
            layout.setStaticPositions();
            //layout.setStaticColorsRGB();

            const size_t kNumVertices = 4;
            vector<Vec3f> positions(kNumVertices);
            // CCW
            // #3: -271.0, 10485.0 ---- #2: 4129.0, 10485.0
            //
            // #1: -271.0, -1452.0 ---- #0: 4129.0, -1452.0 
            positions[0] = Vec3f(4129.0, -1452.0, 33626);
            positions[1] = Vec3f(-271.0, -1452.0, 33626);
            positions[2] = Vec3f(-271.0, 10485.0, 33626);
            positions[3] = Vec3f(4129.0, 10485.0, 33626);
            for (size_t i=0; i<kNumVertices; i++)
            {
                positions[i] *= SPHERE_UNIT_RATIO;
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
            mVboWall.bufferPositions( positions );
            mVboWall.bufferTexCoords2d( 0, texCoords );
            //mVboWall.bufferColorsRGB(colors);
        }
    }

    void resize( ResizeEvent event )
    {
        App::resize( event );
        mArcball.setWindowSize( getWindowSize() );
        mArcball.setCenter( Vec2f( getWindowWidth() / 2.0f, getWindowHeight() / 2.0f ) );
        mArcball.setRadius( 150 );
    }

    void mouseDown(MouseEvent event)
    {
        if( event.isAltDown() )
            mMayaCam.mouseDown( event.getPos() );
        else
            mArcball.mouseDown( event.getPos() );
    }

    void mouseDrag(MouseEvent event)
    {	
        if( event.isAltDown() )
            mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
        else
            mArcball.mouseDrag( event.getPos() );
    }

    void onOscMessage(const osc::Message* msg)
    {
        const string& addr = msg->getAddress();

        if (addr == "/pull")
        {
            int file_idx = msg->getArgAsInt32(0);
        }
    }

    void keyUp(KeyEvent event)
    {
        switch (event.getCode())
        {
        case KeyEvent::KEY_ESCAPE:
            {
                quit();
            }break;
        case KeyEvent::KEY_SPACE:
            {
                mArcball.resetQuat();
            }break;
        case KeyEvent::KEY_h:
            {
                mParams.show(!mParams.isVisible());
            }break;
        }
    }

    void update()
    {
        float delta = getElapsedSeconds() - mPrevSec;
        mPrevSec = getElapsedSeconds();
        if (mNextAnim != mCurrentAnim)
        {
            mAnims[mCurrentAnim].reset();
            mCurrentAnim = mNextAnim;
        }
        mAnims[mCurrentAnim].update(delta * max<float>(ANIM_SPEED, 0));

        const Surface& suf = mAnims[mCurrentAnim].getFrame();
        Vec3f aabbSize = mAABB.getSize();
        Vec3f aabbMin = mAABB.getMin();

        int32_t width = suf.getWidth();
        int32_t height = suf.getHeight();

        BOOST_FOREACH(Led& led, mLeds)
        {
            float cx = (led.pos.z/* - aabbMin.z*/) / aabbSize.z;
            float cy = (led.pos.x/* - aabbMin.x*/) / aabbSize.x;
            uint8_t red = *suf.getDataRed(Vec2i(width * cx, height * cy));
            led.value = red / 255.f;
        }

        //
        if (mNextAnimWall != mCurrentAnimWall)
        {
            mAnimWalls[mCurrentAnimWall].reset();
            mCurrentAnimWall = mNextAnimWall;
        }
        mAnimWalls[mCurrentAnimWall].update(delta * max<float>(ANIM_WALL_SPEED, 0));
    }

    void draw()
    {
        gl::enableDepthRead();
        gl::enableDepthWrite();

        gl::clear(ColorA::gray(43 / 255.f));
        gl::setMatrices(mMayaCam.getCamera());

        gl::rotate( mArcball.getQuat() );

        if (COORD_FRAME_VISIBLE)
        {
            gl::drawCoordinateFrame(50.0f);
        }

        gl::pushModelView();
        {
            gl::translate(mAABB.getSize() * -0.5f);
            gl::scale(-1, 1, 1);

            // lines
            gl::enableAlphaBlending();
            gl::disableDepthWrite();
            gl::color(ColorA::gray(76 / 255.f, 76 / 255.f));
            BOOST_FOREACH(const Led& led, mLeds)
            {
                gl::drawLine(led.pos, Vec3f(led.pos.x, CEILING_HEIGHT, led.pos.z));
            }

            // spheres
            gl::enableDepthWrite();
            BOOST_FOREACH(const Led& led, mLeds)
            {
                gl::color(ColorA::gray(1.0f, constrain<float>(led.value, SPHERE_MIN_ALPHA, 1.0f)));
                gl::drawSphere(led.pos, SPHERE_RADIUS);
            }
            gl::disableAlphaBlending();

            // wall
            gl::Texture tex = mAnimWalls[mCurrentAnimWall].getTexture();
            tex.enableAndBind();
            gl::draw(mVboWall);
            tex.disable();
        }
        gl::popModelView();

        if (ANIM_COUNT_VISIBLE)
        {
            gl::setMatricesWindow(getWindowSize());
            gl::drawString(toString(mAnims[mCurrentAnim].index), Vec2f(10, 10));
            gl::drawString(toString(mAnimWalls[mCurrentAnimWall].index), Vec2f(10, 30));
        }

        mParams.draw();
    }

    bool loadAnimFromDir( fs::path dir, Anim& aAnim ) 
    {
        aAnim.name = dir.filename().string();
        for ( fs::directory_iterator dir_iter(dir); dir_iter != end_iter; ++dir_iter)
        {
            if (!fs::is_regular_file(*dir_iter))
                continue;
            Surface suf = loadImage(*dir_iter);
            if (suf)
            {
                aAnim.frames.push_back(suf);
            }
        }

        if (aAnim.frames.empty())
        {
            return false;
        }

        return true;
    }

private:
    params::InterfaceGl mParams;
    osc::Listener   mListener;
    vector<Led>     mLeds;
    MayaCamUI		mMayaCam;
    Arcball         mArcball;
    AxisAlignedBox3f mAABB;

    vector<Anim>    mAnims;
    vector<Anim>    mAnimWalls;

    int             mCurrentAnim;
    int             mNextAnim;

    // TODO: merge
    int             mCurrentAnimWall;
    int             mNextAnimWall;

    float           mPrevSec;

    gl::VboMesh     mVboWall;
};

CINDER_APP_BASIC(CiApp, RendererGl)
