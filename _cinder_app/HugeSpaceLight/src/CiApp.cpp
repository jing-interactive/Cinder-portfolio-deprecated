#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

#include "cinder/osc/OscListener.h"
#include "../../../_common/MiniConfig.h"
#include <fstream>
#include <boost/foreach.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

struct CiApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        readConfig();
        
        settings->setWindowPos(0, 0);
        settings->setWindowSize(WIN_WIDTH, WIN_HEIGHT);
    }

    void setup()
    {
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);
        mParams.hide();

        mListener.setup(OSC_PORT);
        mListener.registerMessageReceived(this, &CiApp::onOscMessage);

        ifstream ifs(getAssetPath("leds.txt").string().c_str());
        int id;
        float x, y, z;

        while (ifs >> id >> x >> z >> y)
        {
            Vec3f pos(x, y, z);
            pos *= SPHERE_POS_RATIO;
            mLedPositions.push_back(pos);
            mMaxBound.x = max<float>(mMaxBound.x, pos.x);
            mMaxBound.y = max<float>(mMaxBound.y, pos.y);
            mMaxBound.z = max<float>(mMaxBound.z, pos.z);
        }

        CameraPersp initialCam;
        initialCam.setPerspective(CAM_FOV, getWindowAspectRatio(), 0.1f, 10000.0f);
        initialCam.lookAt(Vec3f(mMaxBound.x / 2, mMaxBound.y * 0.3f, - mMaxBound.z * 0.1f), Vec3f(mMaxBound.x / 2, mMaxBound.y / 2, mMaxBound.z));
        mMayaCam.setCurrentCam(initialCam);

        gl::enableDepthRead();
        gl::enableDepthWrite();
    }

    void mouseDown(MouseEvent event)
    {
        mMayaCam.mouseDown(event.getPos());
    }

    void mouseDrag(MouseEvent event)
    {
        mMayaCam.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
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
        if (event.getCode() == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void update()
    {
    
    }

    void draw()
    {
        gl::clear(ColorA::black());
        gl::setMatrices(mMayaCam.getCamera());

        gl::color(Color::white());
        std::for_each(mLedPositions.begin(), mLedPositions.end(),
            std::bind(gl::drawSphere, _1, SPHERE_RADIUS, 12));

        mParams.draw();
    }

private:
    params::InterfaceGl mParams;
    osc::Listener   mListener;
    vector<Vec3f>   mLedPositions;
    MayaCamUI		mMayaCam;
    Vec3f           mMaxBound;
};

CINDER_APP_BASIC(CiApp, RendererGl)
