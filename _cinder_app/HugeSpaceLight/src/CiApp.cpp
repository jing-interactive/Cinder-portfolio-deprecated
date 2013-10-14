#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

#include "cinder/osc/OscListener.h"
#include "../../../_common/MiniConfig.h"

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

        mListener.setup(OSC_PORT);
        mListener.registerMessageReceived(this, &OscApp::onOscMessage);
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

        mParams.draw();
    }

private:
    params::InterfaceGl mParams;
    osc::Listener   mListener;
};

CINDER_APP_BASIC(CiApp, RendererGl)
