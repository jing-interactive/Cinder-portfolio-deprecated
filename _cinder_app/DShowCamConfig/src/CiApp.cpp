#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Capture.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"

#include "msw/videoInput/videoInput.h"
#include <DShow.h>

#include "../../../_common/MiniConfig.h"

using namespace ci;
using namespace ci::app;
using namespace std;

extern videoInput* gVideoInput;

int gBrightness = -1;
int gContrast = -1;
int gSaturation = -1;
int gWhiteBalance = -1;
int gGain = -1;
int gExposure = -1;
int gFocus = -1;

int WIN_WIDTH = 800;
int WIN_HEIGHT = 600;
int CAM_W = 800;
int CAM_H = 600;

#pragma warning(disable: 4244)

void resetValues()
{
    Brightness = 130;
    Contrast = 130;
    Saturation = 110;
    WhiteBalance = 4000;
    Gain = 160;
    Exposure = -5;
    Focus = 16;
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
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight() + 30));
        mParams.addButton("RESET", resetValues);
        setupConfigUI(&mParams);

        vector<Capture::DeviceRef> devices(Capture::getDevices());
        if (devices.empty())
        {
            console() << "No camera device connected." << endl;
            quit();
            return;
        }

        for (vector<Capture::DeviceRef>::const_iterator deviceIt = devices.begin(); deviceIt != devices.end(); ++deviceIt)
        {
            Capture::DeviceRef device = *deviceIt;
            console() << "Found Device " << device->getName() << " ID: " << device->getUniqueId() << endl;

            if (device->checkAvailable())
            {
                mDevices.push_back(device);
                mDeviceNames.push_back(device->getName());
            }
        }
        ADD_ENUM_TO_INT(mParams, DEVICE_ID, mDeviceNames);

        mDeviceId = -1;

        mIsOneShot = getArgs().size() > 1;
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
        DEVICE_ID = constrain<int>(DEVICE_ID, 0, mDevices.size() - 1);
        if (mDeviceId != DEVICE_ID)
        {
            mDeviceId = DEVICE_ID;
            if (mCapture)
            {
                mCapture.stop();
            }
            mCapture = Capture(CAM_W, CAM_H, mDevices[DEVICE_ID]);
            mCapture.start();
            CAM_W = mCapture.getWidth();
            CAM_H = mCapture.getHeight();

            if (!mIsOneShot)
            {
                long a, b, c, d, e, f;
                gVideoInput->getVideoSettingFilter(DEVICE_ID, VideoProcAmp_Brightness, a, b, c, f, d, e);Brightness = f;
                gVideoInput->getVideoSettingFilter(DEVICE_ID, VideoProcAmp_Contrast, a, b, c, f, d, e);Contrast = f;
                gVideoInput->getVideoSettingFilter(DEVICE_ID, VideoProcAmp_Saturation, a, b, c, f, d, e);Saturation = f;
                gVideoInput->getVideoSettingFilter(DEVICE_ID, VideoProcAmp_WhiteBalance, a, b, c, f, d, e);WhiteBalance = f;
                gVideoInput->getVideoSettingFilter(DEVICE_ID, VideoProcAmp_Gain, a, b, c, f, d, e);Gain = f;

                gVideoInput->getVideoSettingCamera(DEVICE_ID, CameraControl_Exposure, a, b, c, f, d, e);Exposure = f;
                gVideoInput->getVideoSettingCamera(DEVICE_ID, CameraControl_Focus, a, b, c, f, d, e);Focus = f;
            }
            // TODO: placeholder text
            mCaptureTex = gl::Texture();
        }

        if (gBrightness != Brightness)
        {
            gBrightness = Brightness;
            gVideoInput->setVideoSettingFilter(DEVICE_ID, VideoProcAmp_Brightness,  Brightness);
        }
        if (gContrast != Contrast)
        {
            gContrast = Contrast;
            gVideoInput->setVideoSettingFilter(DEVICE_ID, VideoProcAmp_Contrast,    Contrast);
        }
        if (gSaturation != Saturation)
        {
            gSaturation = Saturation;
            gVideoInput->setVideoSettingFilter(DEVICE_ID, VideoProcAmp_Saturation,  Saturation);
        }
        if (gWhiteBalance != WhiteBalance)
        {
            gWhiteBalance = WhiteBalance;
            gVideoInput->setVideoSettingFilter(DEVICE_ID, VideoProcAmp_WhiteBalance,WhiteBalance);
        }
        if (gGain != Gain)
        {
            gGain = Gain;
            gVideoInput->setVideoSettingFilter(DEVICE_ID, VideoProcAmp_Gain,        Gain);
        }
        
        if (gExposure != Exposure)
        {
            gExposure = Exposure;
            gVideoInput->setVideoSettingCamera(DEVICE_ID, CameraControl_Exposure,   Exposure);
        }
        if (gFocus != Focus)
        {
            gFocus = Focus;
            gVideoInput->setVideoSettingCamera(DEVICE_ID, CameraControl_Focus,      Focus);
        }

        if (mCapture.checkNewFrame())
        {
            mCaptureTex = mCapture.getSurface();
        }

        if (mIsOneShot)
        {
            console() << "Quit" << endl;
            quit();
        }
    }

    void draw()
    {
        gl::setMatricesWindow(getWindowSize());
        gl::clear(ColorA::black());

        if (mCaptureTex)
        {
            gl::draw(mCaptureTex, getWindowBounds());
        }

        mParams.draw();
    }

private:
    params::InterfaceGl mParams;

    vector<Capture::DeviceRef>  mDevices;
    vector<string>              mDeviceNames;
    int                         mDeviceId;
    Capture                     mCapture;
    gl::Texture                 mCaptureTex;

    bool                        mIsOneShot;
};

CINDER_APP_BASIC(CiApp, RendererGl)
