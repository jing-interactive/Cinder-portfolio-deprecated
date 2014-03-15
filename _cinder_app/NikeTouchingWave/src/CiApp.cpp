#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Serial.h"

#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"
#include "../../../_common/AssetManager.h"

#include <boost/foreach.hpp> 

using namespace ci;
using namespace ci::app;
using namespace std;

#pragma warning(disable: 4244)

const int kButtonCount = 20;

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
        const std::vector<Serial::Device>& devices = Serial::getDevices();
        BOOST_FOREACH(const Serial::Device& dev, devices)
        {
            console() << "Device: " << dev.getName() << endl;
        }

        if (!devices.empty())
        {
            try
            {
                mSerial = Serial(devices[0], 9600);
            }
            catch (...)
            {
                console() << "There was an error initializing the serial device!" << endl;
                quit();
            }
        }

        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);

        for (int i=0; i<kButtonCount; i++)
        {
            string name = "btn_" + i;
            mParams.addButton(name, bind(&CiApp::onButtonClick, this, i));
        }
    }

    void onButtonClick(int id)
    {
        console() << "Hit " << id << endl;
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
        mSerial.readByte();
    }

    void draw()
    {
        gl::setMatricesWindow(getWindowSize());
        gl::clear(ColorA::black());

        mParams.draw();
    }

private:
    params::InterfaceGl mParams;
    Serial              mSerial;
};

CINDER_APP_BASIC(CiApp, RendererGl)
