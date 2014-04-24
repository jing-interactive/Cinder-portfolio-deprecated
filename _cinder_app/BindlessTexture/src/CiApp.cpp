#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/gl/GlslProg.h"

#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"
#include "../../../_common/AssetManager.h"
#include "../../../_common/glplus.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#pragma warning(disable: 4244)

struct CiApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        readConfig();
        
        settings->setWindowSize(WIN_WIDTH, WIN_HEIGHT);
    }

    void setup()
    {
        console() << glGetString(GL_VENDOR) << endl;
        console() << glGetString(GL_VERSION) << endl;
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);

#if 0
        shader.vertexShader(glsl(
            attribute vec2 vertex;
            varying vec2 coord;
            void main() {
                coord = vertex * 0.5 + 0.5;
                gl_Position = vec4(vertex, 0.0, 1.0);
            }
        )).fragmentShader(glsl(
            varying vec2 coord;
            void main() {
                gl_FragColor = vec4(coord, 0.0, 1.0);
            }
        )).link();
#else
        shader.vertexShader(RAW_STRING(
            #version 400\n

            in vec2 vertex;
            out vec2 coord;

            void main() {
                coord = vertex * 0.5 + 0.5;
                gl_Position = vec4(vertex, 0.0, 1.0);
            }
        )).fragmentShader(RAW_STRING(
            #version 400\n

            in vec2 coord;
            layout(location = 0) out vec4 fragColor;

            void main() {
                fragColor = vec4(coord, 0.0, 1.0);
            }
        )).link();
#endif
        quad << Vec2f(-1, -1) << Vec2f(1, -1) << Vec2f(-1, 1) << Vec2f(1, 1);
        quad.upload();
        layout.create(shader, quad).attribute<float>("vertex", 2).check();
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
        gl::setMatricesWindow(getWindowSize());
        gl::clear(ColorA::black());

        shader.use();
        layout.draw(GL_TRIANGLE_STRIP);
        shader.unuse();

        mParams.draw();
    }

private:
    params::InterfaceGl mParams;
    glplus::Shader shader;
    glplus::Buffer<Vec2f> quad;
    glplus::VAO layout;
};

CINDER_APP_BASIC(CiApp, RendererGl)
