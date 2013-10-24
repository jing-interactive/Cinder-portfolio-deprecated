#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/Json.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"

#include <boost/foreach.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

typedef function<void(const JsonTree&)> JsonHandler;
typedef pair<string, JsonHandler> NameHandlerPair;
typedef map<string, GLenum> NameEnumMap;

struct Hero
{
    map<string, gl::Texture::Format>        mSamplers;
    map<string, fs::path>                   mImages;
    map<string, gl::Texture>                mTextures;
};

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

        // parse dota2hero-gh-pages/heroes
        mHeroesPath = fs::path(HEROES_PATH);
        fs::directory_iterator end_iter;
        for (fs::directory_iterator dir_iter(mHeroesPath); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_directory(*dir_iter))
            {
				mHeroNames.push_back((*dir_iter).path().filename().string());
            }
    	}

    	mCurrentHero = -1;
        mParams.removeParam("CURRENT_HERO");
    	mParams.addParam("CURRENT_HERO", mHeroNames, &CURRENT_HERO);

        mCurrentTexture = -1;

        // std::functios
#define BIND_PAIR(name, handler) do \
        {\
        JsonHandler childF = bind(&CiApp::handler, this, _1);\
        JsonHandler parentF = bind(&CiApp::handleChildren, this, _1, childF);\
        mCategories.push_back(make_pair(name, parentF));\
        } while (0);
        BIND_PAIR("buffers",        handleNothing);
        BIND_PAIR("bufferViews",    handleNothing);
        BIND_PAIR("images",         handleImage);
        BIND_PAIR("videos",         handleNothing);
        BIND_PAIR("samplers",       handleSampler);
        BIND_PAIR("textures",       handleTexture);
        BIND_PAIR("shaders",        handleNothing);
        BIND_PAIR("programs",       handleNothing);
        BIND_PAIR("techniques",     handleNothing);
        BIND_PAIR("materials",      handleNothing);
        BIND_PAIR("indices",        handleNothing);
        BIND_PAIR("attributes",     handleNothing);
        BIND_PAIR("meshes",         handleNothing);
        BIND_PAIR("cameras",        handleNothing);
        BIND_PAIR("lights",         handleNothing);
        BIND_PAIR("skins",          handleNothing);
        BIND_PAIR("nodes",          handleNothing);
        BIND_PAIR("scenes",         handleNothing);
        BIND_PAIR("animations",     handleNothing);
#undef BIND_PAIR  

#define ADD_ENUM(item) mGlNameMap[#item] = GL_##item
        // texture format
        ADD_ENUM(RGB);
        ADD_ENUM(RGBA);

        // texture target
        ADD_ENUM(TEXTURE_1D);
        ADD_ENUM(TEXTURE_2D);
        ADD_ENUM(TEXTURE_3D);

        // texture wrap mode
        ADD_ENUM(REPEAT);
        ADD_ENUM(CLAMP_TO_EDGE);

        // sampler filter
        ADD_ENUM(LINEAR);
        ADD_ENUM(LINEAR_MIPMAP_LINEAR);
#undef ADD_ENUM
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
        if (mCurrentHero != CURRENT_HERO)
        {
            mCurrentHero = CURRENT_HERO;
            if (loadHero(mHeroNames[mCurrentHero]))
            {
                mParams.removeParam("CURRENT_TEXTURE");
                mParams.addParam("CURRENT_TEXTURE", mTextureNames, &CURRENT_TEXTURE);
            }
        }

        if (mCurrentTexture != CURRENT_TEXTURE)
        {
            mCurrentTexture = CURRENT_TEXTURE;
        }
    }

    void draw()
    {
        gl::clear(ColorA::black());
        gl::setMatricesWindow(getWindowSize());

        gl::draw(mHero.mTextures[mTextureNames[mCurrentTexture]], Vec2f(100, 100));

        mParams.draw();
    }

private:
    bool loadHero(const std::string& heroName)
    {
        fs::path heroRoot = mHeroesPath / heroName;

        fs::path meshPath = heroRoot / (heroName+".json");
        fs::path animPath = heroRoot / "animations.json";
        fs::path mtrlPath = heroRoot / "materials.json";

        if (!fs::exists(meshPath) ||
            !fs::exists(animPath) ||
            !fs::exists(mtrlPath) )
            return false;

        JsonTree heroJsonRoot = JsonTree(loadFile(meshPath));

        mHero = Hero();
        mTextureNames.clear();

        // TODO: exception safe
        BOOST_FOREACH(const NameHandlerPair& aPair, mCategories)
        {
            string name = aPair.first;
            console() << "// " << name << endl;
            JsonHandler handler = aPair.second;

            if (heroJsonRoot.hasChild(name))
            {
               handler(heroJsonRoot.getChild(name)); 
            }
            console() << endl;
        }

        return true;
    }

    void handleNothing(const JsonTree& tree)
    {
    }

    void handleChildren(const JsonTree& tree, const JsonHandler& handler)
    {
        BOOST_FOREACH(const JsonTree& child, tree.getChildren())
        {
#ifdef _DEBUG
            console() << child.getKey() << ":" << child.getChildren().size() << endl;
#endif
            handler(child);
        }
    }

    //"image_0": {
    //    "path": "textures/abaddon_body_color.png"
    //},
    void handleImage(const JsonTree& tree)
    {
        mHero.mImages[tree.getKey()] = getAbsolutePath(tree.getChild("path").getValue());
    }

    //"sampler_0": {
    //    "magFilter": "LINEAR",
    //    "minFilter": "LINEAR_MIPMAP_LINEAR",
    //    "wrapS": "REPEAT",
    //    "wrapT": "REPEAT"
    //},
    void handleSampler(const JsonTree& tree)
    {
        gl::Texture::Format& format = mHero.mSamplers[tree.getKey()];
        format.setMagFilter(getGlEnum(tree, "magFilter"));
        format.setMinFilter(getGlEnum(tree, "minFilter"));
        format.setWrapS(getGlEnum(tree, "wrapS"));
        format.setWrapT(getGlEnum(tree, "wrapT"));
        format.enableMipmapping();
    }

    //"Map #19": {
    //    "format": "RGBA",
    //    "internalFormat": "RGBA",
    //    "sampler": "sampler_0",
    //    "source": "image_0",
    //    "target": "TEXTURE_2D"
    //},
    void handleTexture(const JsonTree& tree)
    {
        fs::path imagePath = mHero.mImages[tree.getChild("source").getValue()];
        Surface surf = loadImage(imagePath);

        gl::Texture::Format& format = mHero.mSamplers[tree.getChild("sampler").getValue()];
        format.setTarget(getGlEnum(tree, "target"));
        format.setInternalFormat(getGlEnum(tree, "internalFormat"));

        mHero.mTextures[tree.getKey()] = gl::Texture(surf, format);

        mTextureNames.push_back(tree.getKey());
    }

    fs::path getAbsolutePath(const string& relativePath)
    {
        return mHeroesPath / mHeroNames[mCurrentHero] / relativePath;
    }

    GLenum getGlEnum(const JsonTree& tree, const string& childKeyName)
    {
        const string& childKeyValue = tree.getChild(childKeyName).getValue();
        return getGlEnum(childKeyValue);
    }

    GLenum getGlEnum(const string& name)
    {
        NameEnumMap::const_iterator it = mGlNameMap.find(name);
        if (it != mGlNameMap.end())
        {
            return it->second;
        }
        
        console() << "Unsupported enum: " << name << endl;
        // report each unregistered enum for once
        mGlNameMap.insert(make_pair(name, GL_NONE));
        return GL_NONE;
    }

private:
    fs::path                mHeroesPath;
    params::InterfaceGl     mParams;
    vector<NameHandlerPair> mCategories;
    NameEnumMap             mGlNameMap;

    Hero                    mHero;

    // params
    vector<string>          mHeroNames;
    int 		            mCurrentHero;
    vector<string>          mTextureNames;
    int                     mCurrentTexture;
};

CINDER_APP_BASIC(CiApp, RendererGl)
