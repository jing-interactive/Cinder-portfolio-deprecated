#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Capture.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"

using namespace ci;
using namespace ci::app;
using namespace std;

static void updateTextureFromSurface(gl::Texture& dst, const Surface& src)
{
    if (dst && src.getSize() == dst.getSize())
        dst.update(src);
    else
        dst = gl::Texture(src);
}

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

        vector<Capture::DeviceRef> devices(Capture::getDevices());
        if (devices.empty())
        {
            console() << "No valid capture device." << endl;
            quit();
        }

        size_t deviceId = 0;
        try
        {
            mCapture = Capture(CAM_WIDTH, CAM_HEIGHT, devices[deviceId]);
            mCapture.start();
        }
        catch (CaptureExc &)
        {
            console() << "Unable to initialize device: " << devices[deviceId]->getName() << endl;
            quit();
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
        if (mCapture.checkNewFrame())
        {
            mCaptureSurf = mCapture.getSurface();
            updateTextureFromSurface(mCaptureTex, mCaptureSurf);
        }
    }

    void draw()
    {
        gl::setMatricesWindow(getWindowSize());
        gl::clear(ColorA::black());

        if (mCaptureSurf)
        {
            gl::draw(mCaptureTex, getWindowBounds());
        }

        mParams.draw();
    }

private:
    params::InterfaceGl mParams;
    Capture             mCapture;
    gl::Texture         mCaptureTex;
    Surface8u           mCaptureSurf;
};

CINDER_APP_BASIC(CiApp, RendererGl)
