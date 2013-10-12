#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Rand.h"
#include "cinder/Camera.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"

#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"

using namespace ci;
using namespace ci::app;
using namespace std;

struct CiApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        settings->setWindowPos(0, 0);
        settings->setWindowSize(WIN_WIDTH, WIN_HEIGHT);

        readConfig();
    }

    void setup()
    {
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);

        mWorldTex = loadImage(loadAsset("texture.png"));

        for (int z = 0; z < kCellCount; z++)
        {
            for (int y = 0; y < kCellCount; y++)
            {
                for (int x = 0; x < kCellCount; x++)
                {
                    float yd = (y - kCellCount * 0.5f + 0.5f) * 0.4;
                    float zd = (z - kCellCount * 0.5f + 0.5f) * 0.4;
                    mCells[z][y][x] = randInt(16);
                    if (randFloat() > math<float>::sqrt(math<float>::sqrt(yd * yd + zd * zd)) - 0.8f)
                    {
                        mCells[z][y][x] = 0;
                    }
                }
            }
        }
    }

    void resize( ResizeEvent event )
    {
        mCam.lookAt( Vec3f( 0.0f, 0.0f, 500.0f ), Vec3f::zero() );
        mCam.setPerspective( 60, getWindowAspectRatio(), 1, 5000 );
        gl::setMatrices( mCam );
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

        mWorldTex.bind();
        for (int z = 0; z < kCellCount; z++)
        {
            for (int y = 0; y < kCellCount; y++)
            {
                for (int x = 0; x < kCellCount; x++)
                {
                    int idx = mCells[z][y][x];
                    if (idx > 14)
                    {
                        gl::drawCube(Vec3f(x * CELL_SIZE, y * CELL_SIZE, z * CELL_SIZE), 
                            Vec3f(CELL_SIZE, CELL_SIZE, CELL_SIZE));
                    }
                }
            }
        }
        mParams.draw();
    }

private:
    // opengl
    params::InterfaceGl mParams;
    gl::Texture         mWorldTex;
    CameraPersp         mCam;

private:
    // world
    static const size_t kCellCount = 64;
    int                 mCells[kCellCount][kCellCount][kCellCount];
};

CINDER_APP_BASIC(CiApp, RendererGl)
