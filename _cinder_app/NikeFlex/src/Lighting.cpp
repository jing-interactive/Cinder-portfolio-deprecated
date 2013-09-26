#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const int kLedCount = 83;
int currentLed = 0;

struct OscApp : public AppBasic
{
    void prepareSettings(Settings *settings)
    {
        settings->setTitle("Lighting");
        settings->setBorderless();
        settings->setWindowPos(0, 0);
        settings->setWindowSize(640, 480);
    }

    void keyUp(KeyEvent event)
    {
        if (event.getCode() == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
        else if (event.getCode() == KeyEvent::KEY_RIGHT)
        {
            currentLed++;
            if (currentLed > kLedCount - 1)
            {
                currentLed = 0;
            }
        }
        else if (event.getCode() == KeyEvent::KEY_LEFT)
        {
            currentLed--;
            if (currentLed < 0)
            {
                currentLed = kLedCount - 1;
            }
        }
    }

    void draw()
    {
        gl::clear(ColorA::black());
        gl::color(Color::white());
        gl::drawPoint(Vec2f(currentLed, 0));
        gl::pushModelView();
            gl::scale(2, 2, 2);
            gl::drawString(toString(currentLed), Vec2f(30, 30));
        gl::popModelView();
    }
};

CINDER_APP_BASIC(OscApp, RendererGl)
